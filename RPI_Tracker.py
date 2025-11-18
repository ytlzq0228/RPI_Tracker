from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates

import threading
import json
import time
import os
import configparser
import requests
from gps3 import gps3
from datetime import datetime

# ---------------- GPIO / MockGPIO ----------------
try:
	import RPi.GPIO as GPIO
except (ImportError, ModuleNotFoundError):
	class MockGPIO:
		BCM = BOARD = IN = OUT = HIGH = LOW = None

		@staticmethod
		def setwarnings(*args, **kwargs):
			pass
		@staticmethod
		def setmode(*args, **kwargs):
			pass
		@staticmethod
		def setup(*args, **kwargs):
			pass
		@staticmethod
		def input(*args, **kwargs):
			return 0
		@staticmethod
		def output(*args, **kwargs):
			pass
		@staticmethod
		def cleanup(*args, **kwargs):
			pass
	GPIO = MockGPIO()

# ---------------- 配置读取 ----------------
CONFIG_FILE = '/etc/GPS_config.ini'
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO_STARTUP_PIN = 21
GPIO_LOG_ERROR_PIN = 16
GPIO.setup(GPIO_STARTUP_PIN, GPIO.OUT)
GPIO.setup(GPIO_LOG_ERROR_PIN, GPIO.OUT)

GPIO.output(GPIO_STARTUP_PIN, True)

# ---------------- FastAPI & 模板 ----------------
app = FastAPI()
templates = Jinja2Templates(directory="web_templates")

# 如果有静态资源，可以这样挂载（按需）
# app.mount("/static", StaticFiles(directory="static"), name="static")

# ---------------- GPSD 连接 ----------------
gps_socket = gps3.GPSDSocket()
data_stream = gps3.DataStream()
GPSd_host = config['GPS_Config']['GPSd_host']
GPSd_port = config['GPS_Config']['GPSd_port']
gps_socket.connect(host=GPSd_host, port=GPSd_port)
gps_socket.watch(enable=True, gpsd_protocol='json')

# ---------------- 数据结构与工具函数 ----------------
def get_constellation(prn):
	# PRN 号与星座映射
	constellation_map = {
		'GPS': range(1, 32),
		'SBAS': range(33, 64),
		'GL': range(65, 97),
		'GA': range(301, 337),
		'BD': range(401, 431),
		'QZSS': range(193, 198),
		'IRNSS': range(401, 408),
		'WAAS': range(133, 139),
		'EGNOS': range(120, 139),
		'GAGAN': range(127, 129),
		'MSAS': range(129, 138),
	}

	for constellation, prns in constellation_map.items():
		if prn in prns:
			return f"{constellation}_{prn}"
	return f"Unknow_{prn}"

gps_data_cache = {
	'SNR': {'satellites': [],'sat_map': {}},
	'TPV': {},
	'Path': {},
	'status_data': {},
	"TPV_Raw_data": None
}

# ---------------- 后台线程：GPS 数据 ----------------

def update_gps_data():
	buffer = ""
	while True:
		for new_data in gps_socket:
			if not new_data:
				continue

			# gps3 可能给的是 bytes，也可能是 str，先统一成 str
			if isinstance(new_data, bytes):
				new_data = new_data.decode("utf-8", errors="ignore")

			# 追加到缓冲区
			buffer += new_data

			# 只在缓冲里有换行时才做拆分
			while "\n" in buffer:
				line, buffer = buffer.split("\n", 1)
				line = line.strip()
				if not line or line[:1]!="{":
					continue
				try:
					data_json=json.loads(line)
				except Exception as e:
					print("unpack error:", e)
					print("bad line:", repr(line))
					continue

				# SNR / SKY

				if data_json.get('class') == 'SKY' and 'satellites' in data_json:
					now_ts = time.time()
					for i in data_json['satellites']:
						prn = get_constellation(i['PRN'])
						gps_data_cache['SNR']['sat_map'][prn] = {
							'PRN': prn,
							'ss': i.get('ss', 0),
							'used': i.get('used', False),
							'last_seen': now_ts,
						}
				

					EXPIRE_SECONDS = 60
					last_sat_map= {
						prn: sat for prn, sat in gps_data_cache['SNR']['sat_map'].items()
						if now_ts - sat['last_seen'] <= EXPIRE_SECONDS
					}
					gps_data_cache['SNR']['sat_map']=last_sat_map
					gps_data_cache['SNR']['satellites'] = list(gps_data_cache['SNR']['sat_map'].values())
					#print(gps_data_cache['SNR']['satellites'])



				# TPV
				
				if data_json.get('class') == 'TPV':
					status_data = {}
					status_data['Sat_Qty'] = len(gps_data_cache['SNR']['satellites'])
					gps_data_cache['TPV_Raw_data'] = data_json
					gps_data_cache['TPV_Raw_data']['Sat_Qty'] = len(gps_data_cache['SNR']['satellites'])

					for i in ['alt', 'track', 'magtrack', 'magvar', 'time', 'speed']:
						if i in data_json:
							status_data[i] = data_json[i]
						else:
							status_data[i] = 0

					mode_map = {0: "Unknown",1: "no fix",2: "Normal Mode 2D",3: "Normal Mode 3D",}
					status_map = {0: "Unknown",1: "Normal",2: "DGPS",3: "RTK FIX",4: "RTK FLOAT",5: "DR FIX",6: "GNSSDR",7: "Time (surveyed)",8: "Simulated",9: "P(Y)",}

					if data_json.get('status', 1) == 1:
						status_data['status'] = mode_map.get(
							data_json.get('mode', 0), "Unknown"
						)
					else:
						status_data['status'] = status_map.get(
							data_json.get('status', 1), "Unknown"
						)

					# 米/秒 -> km/h
					status_data['speed'] = "%.2f" % (status_data['speed'] * 3.6)

					if status_data['time'] != 0:
						status_data['time'] = datetime.fromisoformat(
							status_data['time'].replace('Z', '+00:00')
						).strftime('%Y-%m-%d %H:%M:%S')

					gps_data_cache['TPV'] = status_data


					# Path
					for i in ['lat', 'lon', 'speed']:
						if i in data_json:
							gps_data_cache['Path'][i] = data_json[i]

					# speed 阶梯化
					step = 5
					if 'speed' not in gps_data_cache['Path']:
						gps_data_cache['Path']['speed'] = 0
					gps_data_cache['Path']['speed'] = max(
						(round(gps_data_cache['Path']['speed'] / step) * step), 0.5
					)
		time.sleep(0.5)

# ---------------- 后台线程：上报线程状态 ----------------
def update_report_status():
	while True:
		traccar_url = "http://127.0.0.1:5051/traccar_status"
		try:
			resp = requests.get(traccar_url, timeout=3)
			#resp.raise_for_status()
	
			data = resp.json()
			report_ts = data.get("report_traccar_timestamp")
	
			if report_ts is None:
				gps_data_cache['status_data']['TraccarReport'] = "report_traccar_timestamp not found in response"
				continue
	
			now_ts = time.time()
			delay = now_ts - float(report_ts)
			gps_data_cache['status_data']['TraccarReport'] = f"{delay:.2f}"
			gps_data_cache['status_data']['TraccarFAILED_QUEUE'] = data.get("FAILED_QUEUE_Len")
			gps_data_cache['status_data']['TRACCAR_REPORT_INTERVAL'] = data.get("TRACCAR_REPORT_INTERVAL")
		except Exception as e:
			print(f"Error fetching log file data: {e}")
			gps_data_cache['status_data']['TraccarReport'] = f"Error fetching traccar_status: {e}"	
		
		

		aprs_url = "http://127.0.0.1:5052/aprs_status"
		try:
			resp = requests.get(aprs_url, timeout=3)
			#resp.raise_for_status()
	
			data = resp.json()
			report_ts = data.get("report_APRS_timestamp")
	
			if report_ts is None:
				gps_data_cache['status_data']['APRSReport'] = "report_APRS_timestamp not found in response"
				continue
	
			now_ts = time.time()
			delay = now_ts - float(report_ts)
			gps_data_cache['status_data']['APRSReport'] = f"{delay:2.0f}"
			gps_data_cache['status_data']['APRS_REPORT_INTERVAL'] = data.get("APRS_REPORT_INTERVAL")
		except Exception as e:
			print(f"Error fetching log file data: {e}")
			gps_data_cache['status_data']['APRSReport'] = f"Error fetching traccar_status: {e}"
		time.sleep(0.5)


# ---------------- FastAPI 生命周期：启动线程 ----------------
def traccar_report_app():
	uvicorn.run("traccar_report:app", host="0.0.0.0", port=5051, reload=False)
def aprs_report_app():
	uvicorn.run("aprs_report:app", host="0.0.0.0", port=5052, reload=False)

@app.on_event("startup")
def startup_event():
	threading.Thread(target=update_gps_data, daemon=True).start()
	threading.Thread(target=update_report_status, daemon=True).start()
	threading.Thread(target=traccar_report_app, daemon=True).start()
	threading.Thread(target=aprs_report_app, daemon=True).start()



# ---------------- 路由 ----------------
@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
	return templates.TemplateResponse("index.html", {"request": request})


@app.get("/snr-data")
async def snr_data():
	return gps_data_cache['SNR']


@app.get("/tpv-data")
async def tpv_data():
	return gps_data_cache['TPV']


@app.get("/path-data")
async def path_data():
	return gps_data_cache['Path']


@app.get("/status-data")
async def status_data():
	return gps_data_cache['status_data']

@app.get("/GPSd-TPV-Raw-data")
async def TPV_Raw_data():
	if "TPV_Raw_data" in gps_data_cache:
		return gps_data_cache['TPV_Raw_data']
	else:
		return None

if __name__ == "__main__":
	import logging
	import uvicorn
	logging.getLogger("uvicorn.access").setLevel(logging.ERROR)
	uvicorn.run(app, host="0.0.0.0", port=5050)