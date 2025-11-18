import sys
import os
import time
import configparser
from datetime import datetime

# 读取配置文件
CONFIG_FILE='/etc/GPS_config.ini'
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

LOG_FILE_PATH=config['SFTP_Config']['LOCAL_LOG_FILE_PATH']
SSID=config['APRS_Config']['SSID']



def save_log(result):
	try:
		print(result)
		now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		LOG_FILE = f"{LOG_FILE_PATH}/{datetime.now().strftime('%Y-%m-%d')}-GPS-{SSID}.log"
		f = open(LOG_FILE,'a')
		f.writelines("\n%s log:%s" %(now,result))
		f.flush()
		f.close()
	except Exception as err:
		return err

def get_cpu_temperature():
	try:
		with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
			temp = int(f.read().strip()) / 1000.0  # 单位是毫摄氏度，需要转换
		return f"{temp:.2f}"
	except Exception as e:
		return 0


def get_uptime():
	try:
		# 根据系统选命令
		if platform.system() == "Darwin":  # macOS
			cmd = ["uptime"]   # macOS 不支持 -p，需要自己解析
		else:
			cmd = ["uptime", "-p"]

		result = subprocess.run(
			cmd,
			stdout=subprocess.PIPE,
			stderr=subprocess.PIPE,
			text=True,
			check=True,   # 非零退出码会抛异常
		)
		return result.stdout.strip()
	except Exception as e:
		# 命令不存在 / 参数非法 / 退出码非 0 都会走这里
		return 0
