import sys
import os
import time
import re
import serial
import configparser
import aprs
import socket
import requests
import math
from datetime import datetime, timezone, timedelta
from utils.utils import save_log,get_cpu_temperature
import threading

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates

CONFIG_FILE='/etc/GPS_config.ini'
VERSION='traccar_260303.01'

# 读取配置文件
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

Test_Flag=config.getboolean('Test_Flag', 'enable')
VIN=config['Traccar_Config']['VIN']
GPS_Device=config['GPS_Config']['GPS_Device']
TRACCAR_ENABLE=config.getboolean('Traccar_Config', 'enable')
IMU_ENABLE = config.getboolean('IMU_Config', 'enable')
DB_ENABLE=config.getboolean('Traccar_Config', 'DB_enable')
TRACCAR_URL=config['Traccar_Config']['TRACCAR_URL']
TRACCAR_REPORT_INTERVAL=int(config['Traccar_Config']['TRACCAR_REPORT_INTERVAL'])
STILL_REPORT_INTERVAL=int(config['Traccar_Config']['STILL_REPORT_INTERVAL'])
STILL_SPEED_THRESHOLD=int(config['Traccar_Config']['STILL_SPEED_THRESHOLD'])
save_log(f"Traccar Repoeter {VERSION} Starting...")
save_log("Traccar Get Config Params:")
save_log(f"Traccar Param Test_Flag:{Test_Flag}")
save_log(f"Traccar Param VIN:{VIN}")
save_log(f"Traccar Param GPS_Device:{GPS_Device}")
save_log(f"Traccar Param TRACCAR_ENABLE:{TRACCAR_ENABLE}")
save_log(f"Traccar Param IMU_ENABLE:{IMU_ENABLE}")
save_log(f"Traccar Param TRACCAR_URL:{TRACCAR_URL}")
save_log(f"Traccar Param TRACCAR_REPORT_INTERVAL:{TRACCAR_REPORT_INTERVAL}")
save_log(f"Traccar Param STILL_REPORT_INTERVAL:{STILL_REPORT_INTERVAL}")
save_log(f"Traccar Param STILL_SPEED_THRESHOLD:{STILL_SPEED_THRESHOLD}")
GPSd_raw_data={}
IMU_Metrics_data={}
report_traccar_timestamp=0
still_report_traccar_timestamp=0
RETRYABLE_HTTP = {408, 429, 500, 502, 503, 504}

if DB_ENABLE:
	from SQL_Lite_Queue import PersistentQueue
	DB_PATH=config['Traccar_Config']['DB_path']
	SEND_QUEUE = PersistentQueue(DB_PATH, maxlen=0)
else:
	from collections import deque
	SEND_QUEUE = deque(maxlen=5000)


TIME_VALID_MIN_YEAR = 2020

def get_boot_id() -> str:
	# time.monotonic() 是自开机起算的，跨重启不可比，用 boot_id 区分
	try:
		with open("/proc/sys/kernel/random/boot_id", "r") as f:
			return f.read().strip()
	except Exception:
		return ""

BOOT_ID = get_boot_id()

def to_iso(dt: datetime) -> str:
	return dt.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")

def system_time_is_valid() -> bool:
	# RTC 丢失时系统时间会回落到 2000-01-01；ntpd 已由 gpsd(SHM) 和网络NTP共同校时，这里只看系统时钟
	try:
		return datetime.now(timezone.utc).year >= TIME_VALID_MIN_YEAR
	except Exception:
		return False

def fix_payload_timestamp(item: dict) -> bool:
	# 校时完成后用单调时钟回推该点的真实产生时刻，保留点与点之间的间隔
	created_mono = item.get("created_mono")
	if not system_time_is_valid() or created_mono is None or item.get("boot_id") != BOOT_ID:
		return False
	elapsed = max(time.monotonic() - float(created_mono), 0.0)
	old_ts = item["payload"].get("timestamp")
	item["payload"]["timestamp"] = to_iso(datetime.now(timezone.utc) - timedelta(seconds=elapsed))
	item["needs_time_fix"] = False
	save_log(f"Traccar Timestamp Fixed {old_ts} -> {item['payload']['timestamp']} elapsed={elapsed:.1f}s")
	return True


def traccar_report():
	global report_traccar_timestamp,still_report_traccar_timestamp
	while True:
		try:

			current_timestamp=time.time()
			now = time.time()
			#按照上报周期生产新点记录
			if current_timestamp - report_traccar_timestamp < TRACCAR_REPORT_INTERVAL:
				time.sleep(0.1)
				continue
			report_traccar_timestamp = current_timestamp
			lat = GPSd_raw_data.get("lat")
			lon = GPSd_raw_data.get("lon")
			if lat is None or lon is None:
				time.sleep(1)
				continue

			# 时间戳：系统时间失效（RTC丢失、尚未校时）时先记假时间并标记，等校时后由消费端修正
			needs_time_fix = not system_time_is_valid()
			ts = to_iso(datetime.now(timezone.utc))
			if int(GPSd_raw_data.get("speed",0)) > STILL_SPEED_THRESHOLD or current_timestamp - still_report_traccar_timestamp > STILL_REPORT_INTERVAL:
				still_report_traccar_timestamp = current_timestamp
			else:
				continue
			
			payload = {
				"id": str(VIN),
				"lat": f"{float(lat):.7f}",
				"lon": f"{float(lon):.7f}",
				"timestamp": ts,
				"deviceTemp": f"{float(get_cpu_temperature()):.1f}",
				"attributes": {},
			}
			# m/s -> knots
			if GPSd_raw_data.get("speed") is not None:
				payload["speed"] = f"{float(GPSd_raw_data['speed']) * 3600 / 1852:.2f}"
	
			if GPSd_raw_data.get("track") is not None:
				payload["bearing"] = f"{float(GPSd_raw_data['track']):.1f}"
	
			if GPSd_raw_data.get("alt") is not None:
				payload["altitude"] = f"{float(GPSd_raw_data['alt']):.1f}"
	
			if GPSd_raw_data.get("eph") is not None:
				eph_val = float(GPSd_raw_data["eph"])
				payload["accuracy"] = f"{min(eph_val, 100):.1f}"
			
			if GPSd_raw_data.get("Sat_Qty") is not None:
				payload["sat"] = GPSd_raw_data.get("Sat_Qty")

			if IMU_Metrics_data.get("status")=="success" and IMU_ENABLE:
				ignore_k_list=["status","type","n"]
				payload.update({k: v for k, v in IMU_Metrics_data.items() if k not in ignore_k_list})

			save_log(f"Payload Producer:{payload}")
			if needs_time_fix:
				save_log(f"Traccar Hold Payload: system time not synced yet, queued until NTP/GPS fixes the clock")
			SEND_QUEUE.append({
				"payload": payload,
				"attempts": 0,
				"next_ts": 0,
				"created_ts": current_timestamp,
				"created_mono": time.monotonic(),
				"boot_id": BOOT_ID,
				"needs_time_fix": needs_time_fix
			})
		except Exception as e:
			save_log(f"Producer error: {e}")

		time.sleep(0.1)

def traccar_consumer():
	while True:
		try:
			if not SEND_QUEUE or len(SEND_QUEUE)==0:
				time.sleep(0.2)
				continue

			now = time.time()
			item = SEND_QUEUE.popleft()
			if item["next_ts"] > now:
				SEND_QUEUE.append(item)
				time.sleep(0.2)
				continue
			if item.get("needs_time_fix"):
				# 还没校时：整条队列挂起，避免把 2000-01-01 的时间戳上报到 traccar
				if not system_time_is_valid():
					SEND_QUEUE.appendleft(item)   # 放回队首，不打乱上报顺序
					time.sleep(1)
					continue
				if not fix_payload_timestamp(item):
					# 上一次开机遗留的点：monotonic 跨重启不可比，真实时间已无法还原
					save_log(f"Traccar Drop unfixable timestamp payload={item['payload']}")
					continue
			payload = item["payload"]

			try:
				resp = requests.post(TRACCAR_URL, data=payload, timeout=3)
				#resp = requests.post(f"{TRACCAR_URL}/api/positions", data=payload, timeout=3)
				if 200 <= resp.status_code < 300:
					save_log(f"Traccar Sent OK payload={payload} queue={len(SEND_QUEUE)}")
				elif resp.status_code in RETRYABLE_HTTP:
					attempts = item["attempts"] + 1
					backoff = min(60, attempts * 5)
					item.update({"attempts": attempts,"next_ts": now + backoff})
					SEND_QUEUE.append(item)
					save_log(f"Traccar Retry HTTP={resp.status_code} attempts={attempts} backoff={backoff}s queue={len(SEND_QUEUE)}")

				else:
					save_log(f"Traccar Drop HTTP={resp.status_code} body={resp.text[:200]}")

			except Exception as e:
				attempts = item["attempts"] + 1
				backoff = min(60, attempts * 5)
				item.update({"attempts": attempts,"next_ts": now + backoff})
				SEND_QUEUE.append(item)
				save_log(f"Traccar Send Error {e} attempts={attempts} backoff={backoff}s queue={len(SEND_QUEUE)}")

		except Exception as loop_err:
			save_log(f"Consumer loop error: {loop_err}")
			time.sleep(1)

def update_GPSd_raw_data():
	global GPSd_raw_data
	while True:
		try:
			while True:
				try:
					#GPSd_raw_data = GNSS_NMAE.Get_GNSS_Position.GPSd(only_raw_data=True)
					url = "http://127.0.0.1:5050/GPSd-TPV-Raw-data"
					resp = requests.get(url, timeout=1)
					if resp.status_code==200:
						GPSd_raw_data = resp.json()
						break  # 成功获取GNSS数据时退出循环
					time.sleep(1)
				except Exception as err:
					save_log(f"Retrying get_gnss_position with {err}")
					time.sleep(1)  # 等待1秒后重试
			time.sleep(0.5)
		except Exception as err:
			save_log(f"update_GPSd_raw_data: {err}")
			#raise
			#切记全部改完了之后这里把raise注释掉，仅调试期间使用

def update_IMU_Metrics_data():
	global IMU_Metrics_data
	while True:
		try:
			while True:
				try:
					url = "http://127.0.0.1:5053/imu/latest"
					resp = requests.get(url, timeout=1)
					if resp.status_code==200:
						IMU_Metrics_data = resp.json()
						break  # 成功获取数据时退出循环
					time.sleep(1)
				except Exception as err:
					save_log(f"Retrying IMU_Metrics_data with {err}")
					time.sleep(1)  # 等待1秒后重试
			time.sleep(0.5)
		except Exception as err:
			save_log(f"update_IMU_Metrics_data: {err}")
			#raise
			#切记全部改完了之后这里把raise注释掉，仅调试期间使用

app = FastAPI()
@app.on_event("startup")
def startup_event():
	GPS_thread = threading.Thread(target=update_GPSd_raw_data, daemon=True, name="update_GPSd_raw_data")
	GPS_thread.start()

	if IMU_ENABLE:
		IMU_thread = threading.Thread(target=update_IMU_Metrics_data, daemon=True, name="update_IMU_Metrics_data")
		IMU_thread.start()

	if TRACCAR_ENABLE:
		threading.Thread(target=traccar_report,daemon=True,name="traccar_producer").start()
		threading.Thread(target=traccar_consumer,daemon=True,name="traccar_consumer").start()

@app.get("/traccar_status")
async def traccar_status():
	return {
		"queue_len": len(SEND_QUEUE),
		"report_traccar_timestamp": report_traccar_timestamp,
		"still_report_traccar_timestamp": still_report_traccar_timestamp,
		"TRACCAR_REPORT_INTERVAL": TRACCAR_REPORT_INTERVAL
	}

if __name__ == '__main__':
	import logging
	import uvicorn
	logging.getLogger("uvicorn.access").setLevel(logging.ERROR)
	uvicorn.run(app, host="0.0.0.0", port=5051)

