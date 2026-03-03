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
from datetime import datetime
from utils.utils import save_log,get_cpu_temperature,get_uptime
import threading
from utils.Radio_GPIO import read_gpio

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.templating import Jinja2Templates

# 设置全局的socket超时时间，例如10秒
socket.setdefaulttimeout(5)

CONFIG_FILE='/etc/GPS_config.ini'
VERSION='aprs_260303.01'

# 读取配置文件
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

Test_Flag=config.getboolean('Test_Flag', 'enable')
SSID=config['APRS_Config']['SSID']
CALLSIGN=config['APRS_Config']['CALLSIGN']
APRS_PASSWORD=config['APRS_Config']['APRS_PASSWORD']
Message=config['APRS_Config']['Message']
SSID_ICON=config['APRS_Config']['ICON']
APRS_Server=config['APRS_Config']['APRS_Server']
APRS_ENABLE=config['APRS_Config']['APRS_ENABLE']
OLED_Enable=config.getboolean('OLED_Config', 'OLED_Enable')
OLED_Address=int(config.get('OLED_Config', 'OLED_Address'), 16)
GPS_Device=config['GPS_Config']['GPS_Device']
Radio_CONTROL_ENABLE=config['GPIO_CONTROL']['enable']
GPIO_PIN=int(config['GPIO_CONTROL']['GPIO_PIN'])
APRS_REPORT_INTERVAL=int(config['APRS_Config']['APRS_REPORT_INTERVAL'])
NMEA_LOG_INTERVAL=int(config['SYNO_Config']['NMEA_LOG_INTERVAL'])
STILL_LOG_INTERVAL=int(config['SYNO_Config']['STILL_LOG_INTERVAL'])
STILL_SPEED_THRESHOLD=int(config['SYNO_Config']['STILL_SPEED_THRESHOLD'])

save_log(f"APRS Repoeter {VERSION} Starting...")
save_log("APRS Get Config Params:")
save_log(f"APRS Param Test_Flag:{Test_Flag}")
save_log(f"APRS Param SSID:{SSID}")
save_log(f"APRS Param CALLSIGN:{CALLSIGN}")
save_log(f"APRS Param APRS_PASSWORD:{APRS_PASSWORD}")
save_log(f"APRS Param Message:{Message}")
save_log(f"APRS Param SSID_ICON:{SSID_ICON}")
save_log(f"APRS Param APRS_Server:{APRS_Server}")
save_log(f"APRS Param APRS_ENABLE:{APRS_ENABLE}")
save_log(f"APRS Param OLED_Enable:{OLED_Enable}")
save_log(f"APRS Param OLED_Address:{OLED_Address}")
save_log(f"APRS Param GPS_Device:{GPS_Device}")
save_log(f"APRS Param Radio_CONTROL_ENABLE:{Radio_CONTROL_ENABLE}")
save_log(f"APRS Param GPIO_PIN:{GPIO_PIN}")
save_log(f"APRS Param APRS_REPORT_INTERVAL:{APRS_REPORT_INTERVAL}")
save_log(f"APRS Param NMEA_LOG_INTERVAL:{NMEA_LOG_INTERVAL}")
save_log(f"APRS Param STILL_LOG_INTERVAL:{STILL_LOG_INTERVAL}")
save_log(f"APRS Param STILL_SPEED_THRESHOLD:{STILL_SPEED_THRESHOLD}")
report_APRS_timestamp=0
GPSd_raw_data={}
def aprs_report():
	global report_APRS_timestamp
	while True:
		try:
			current_timestamp=time.time()
			if current_timestamp-report_APRS_timestamp>=APRS_REPORT_INTERVAL and read_gpio(Radio_CONTROL_ENABLE,GPIO_PIN):
				report_APRS_timestamp=current_timestamp
				#lat, lat_dir,lon,lon_dir,altitude,NMEA_timestamp,speed,course,GNSS_Type,lat_raw,lon_raw,GPS_Source,GPSd_raw_data
				if "lat" not in GPSd_raw_data or "lon" not in GPSd_raw_data:
					#print("APRS None")
					time.sleep(1)
					continue
				# 纬度转换
				decimal_lat = float(GPSd_raw_data.get('lat', 0))
				lat_dir = "N" if decimal_lat >= 0 else "S"
				lat_degrees = int(abs(decimal_lat))
				lat_minutes = (abs(decimal_lat) - lat_degrees) * 60
	
				# 经度转换
				decimal_lon = float(GPSd_raw_data.get('lon', 0))
				lon_dir = "E" if decimal_lon >= 0 else "W"
				lon_degrees = int(abs(decimal_lon))
				lon_minutes = (abs(decimal_lon) - lon_degrees) * 60
	
				# 格式化 APRS 格式
				lat = f"{lat_degrees:02d}{lat_minutes:05.2f}"
				lon = f"{lon_degrees:03d}{lon_minutes:05.2f}"
	
				# 处理海拔数据
				altitude = f"{float(GPSd_raw_data.get('alt', 0)) * 3.28:06.0f}"
	
				# 处理时间戳
				NMEA_timestamp = datetime.strptime(
					GPSd_raw_data.get('time', "1970-01-01T00:00:00.000Z"),
					"%Y-%m-%dT%H:%M:%S.%fZ"
				).strftime("%H%M%S.00")
				# 处理速度和航向
				speed = f"{float(GPSd_raw_data.get('speed', 0)) * 3600 / 1852:03.0f}"
				course = f"{float(GPSd_raw_data.get('track', 0)):03.0f}"
	
				# 处理 GNSS 状态
				status_map = {
					0: "NOT FIX",
					1: "2D FIX",
					2: "3D FIX",
					3: "RTK FIX",
					4: "RTK FLOAT",
					5: "DR FIX"
				}
				GNSS_Type = status_map.get(GPSd_raw_data.get('status', 0), "UNKNOWN")

				frame_text=(f'{SSID}>PYTHON,TCPIP*,qAC,{SSID}:!{lat}{lat_dir}/{lon}{lon_dir}{SSID_ICON}{course}/{speed}/A={altitude} APRS by RPI with GNSS {GNSS_Type} at UTC {NMEA_timestamp} {Message}').encode()
				save_log(frame_text)
				callsign = CALLSIGN.encode('utf-8')
				password = APRS_PASSWORD.encode('utf-8')
				
				# 定义 APRS 服务器地址和端口（字节形式）
				server_host = APRS_Server.encode('utf-8')  # 使用 rotate.aprs2.net 服务器和端口 14580
				
				# 创建 TCP 对象并传入服务器信息
				a = aprs.TCP(callsign, password, servers=[server_host])
				a.start()
				aprs_return=a.send(frame_text)
				if aprs_return==len(frame_text)+2:
					save_log('APRS Report Good Length:%s'%aprs_return)
					disp_update_time=datetime.now()
				else:
					save_log('APRS Report Return:%s Frame Length: %s Retrying..'%(aprs_return,frame_text))
					disp_update_time=datetime.min
			time.sleep(1)  # 按照设定间隔等待
		except Exception as err:
			save_log(f"APRS Report Error: {err}")

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

app = FastAPI()
@app.on_event("startup")
def startup_event():
	GPS_thread = threading.Thread(target=update_GPSd_raw_data, daemon=True, name="update_GPSd_raw_data")
	GPS_thread.start()
	if APRS_ENABLE:
		traccar_thread = threading.Thread(target=aprs_report, daemon=True, name="aprs_report")
		traccar_thread.start()


@app.get("/aprs_status")
async def aprs_status():
	if APRS_ENABLE:
		data = {
			"report_APRS_timestamp": report_APRS_timestamp,
			"APRS_REPORT_INTERVAL": APRS_REPORT_INTERVAL
		}
	else:
		data = {
			"report_APRS_timestamp": "APRS OFF",
			"APRS_REPORT_INTERVAL": APRS_REPORT_INTERVAL
		}	
	return data

if __name__ == '__main__':
	import logging
	import uvicorn
	logging.getLogger("uvicorn.access").setLevel(logging.ERROR)
	uvicorn.run(app, host="0.0.0.0", port=5052)



