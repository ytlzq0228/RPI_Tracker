import os
import time
import json
import requests
import configparser
from pathlib import Path
from typing import Dict, Optional, Tuple, List
from utils.utils import save_log

# ---------------- 配置读取 ----------------
CONFIG_FILE = '/etc/GPS_config.ini'
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

LOG_FILE_PATH = config['SYNO_Config']['local_update_path']	  # 本地目录
NAS = config['SYNO_Config']['remote_host'].rstrip('/')			# 例如 https://nas.ctsdn.com:5001
USER = config['SYNO_Config']['remote_user']
PASS = config['SYNO_Config']['remote_pass']
REMOTE_FILE_PATH = config['SYNO_Config']['remote_dir'].rstrip('/')  # 例如 /home/logs 或 /volume1/share/logs

# ---------------- 行为开关 ----------------
RECURSIVE = True		  # True: 递归遍历 LOG_FILE_PATH
VERIFY_TLS = True		 # 有正确证书就 True；自签证书可改 False 或填 PEM 路径
OVERWRITE = True		  # True: 远端存在时覆盖（但增量模式下通常不会重复上传）
INCREMENTAL_UPDATE = True		# True: 做增量判断
DRY_RUN = False		   # True: 只打印不上传

# 超时
TIMEOUT_GET = 20
TIMEOUT_POST = 120
WAIT_CYCLE = 60

# ---------------- DSM WebAPI 帮助函数 ----------------
def api_get(session: requests.Session, url: str, params: Dict, verify) -> Dict:
	r = session.get(url, params=params, verify=verify, timeout=TIMEOUT_GET)
	r.raise_for_status()
	return r.json()

def api_post(session: requests.Session, url: str, params: Dict, data: Dict, files: Dict, verify) -> Dict:
	r = session.post(url, params=params, data=data, files=files, verify=verify, timeout=TIMEOUT_POST)
	r.raise_for_status()
	# 有时 DSM 返回 text/plain，也可能是 JSON 字符串
	try:
		return r.json()
	except Exception:
		return {"success": False, "raw": r.text}

def login(session: requests.Session) -> str:
	auth_url = f"{NAS}/webapi/auth.cgi"
	params = {
		"api": "SYNO.API.Auth",
		"version": "6",
		"method": "login",
		"account": USER,
		"passwd": PASS,
		"session": "FileStation",
		"format": "sid",
	}
	j = api_get(session, auth_url, params, VERIFY_TLS)
	if not j.get("success"):
		raise RuntimeError(f"Login failed: {j}")
	return j["data"]["sid"]

def logout(session: requests.Session, sid: str) -> None:
	auth_url = f"{NAS}/webapi/auth.cgi"
	params = {
		"api": "SYNO.API.Auth",
		"version": "6",
		"method": "logout",
		"session": "FileStation",
		"_sid": sid
	}
	try:
		j = api_get(session, auth_url, params, VERIFY_TLS)
		# 不强制要求成功
		save_log(f"logout:{j}")
	except Exception as e:
		save_log(f"logout error:{e}")

def upload_file(session: requests.Session, sid: str, local_file: Path, remote_dir: str) -> Dict:
	upload_url = f"{NAS}/webapi/entry.cgi"
	query = {
		"api": "SYNO.FileStation.Upload",
		"version": "2",
		"method": "upload",
		"_sid": sid,
	}
	data = {
		"path": remote_dir,
		"create_parents": "true",
		"overwrite": "true" if OVERWRITE else "false",
	}

	if DRY_RUN:
		return {"success": True, "dry_run": True, "local": str(local_file), "remote_dir": remote_dir}

	with local_file.open("rb") as f:
		files = {"file": (local_file.name, f, "application/octet-stream")}
		return api_post(session, upload_url, query, data, files, VERIFY_TLS)

# ---------------- 远端信息查询（用于增量判断） ----------------
def build_remote_index(session: requests.Session, sid: str, remote_root: str) -> Dict[str, Tuple[int, int]]:
	"""
	仅索引 remote_root 这一层（不递归）：
	返回 dict: { filename: (size, mtime_epoch_sec) }
	"""
	entry_url = f"{NAS}/webapi/entry.cgi"
	params = {
		"api": "SYNO.FileStation.List",
		"version": "2",
		"method": "list",
		"folder_path": remote_root,
		"additional": json.dumps(["time", "size"]),  # 请求额外字段：mtime/atime/ctime + size
		"_sid": sid,
	}
	try:
		j = api_get(session, entry_url, params, VERIFY_TLS)
		if not j.get("success"):
			return {}
		files = j.get("data", {}).get("files", [])
	except Exception:
		return {}

	idx = {}
	for f in files:
		if f.get("isdir"):
			continue
		name = f.get("name")
		size = int(f.get("additional", {}).get("size", 0) or 0)
		# DSM 返回的 time 通常在 additional.time.mtime，单位是秒（epoch）
		mtime = f.get("additional", {}).get("time", {}).get("mtime")
		mtime = int(mtime or 0)
		if name:
			idx[name] = (size, mtime)
	return idx

# ---------------- 本地遍历与映射 ----------------
def iter_local_files(base_dir: Path, recursive: bool = True) -> List[Path]:
	base_dir = base_dir.resolve()
	if recursive:
		return [p for p in base_dir.rglob("*") if p.is_file()]
	else:
		return [p for p in base_dir.iteremote_dir() if p.is_file()]

def remote_dir_for_file(remote_root: str, rel_path: str) -> str:
	# rel_path 可能是 a/b/c.log，远端目录要到 a/b
	parts = rel_path.split("/")
	if len(parts) <= 1:
		return remote_root
	subdir = "/".join(parts[:-1])
	return f"{remote_root}/{subdir}".replace("//", "/")

def should_upload(local_file: Path, remote_idx: Dict[str, Tuple[int, int]], filename: str) -> bool:
	"""
	增量策略：如果远端不存在 -> 上传
			 如果存在 -> 比较 size；若 size 不同 -> 上传
					   size 相同 -> 再比较 mtime（本地 mtime > 远端 mtime + 1s）才上传
	"""
	if filename not in remote_idx:
		return True

	local_stat = local_file.stat()
	local_size = int(local_stat.st_size)
	local_mtime = int(local_stat.st_mtime)

	remote_size, remote_mtime = remote_idx[filename]

	if local_size != remote_size:
		return True

	# 远端 mtime 可能是上传时间，和本地不完全一致，这里只在“本地更新明显更晚”才上传
	if local_mtime > remote_mtime + 1:
		return True

	return False

# ---------------- 主流程 ----------------
def dsm_upload_files():
	local_log_base_dir = Path(LOG_FILE_PATH)
	if not local_log_base_dir.exists() or not local_log_base_dir.is_dir():
		raise RuntimeError(f"LOG_FILE_PATH not a directory: {local_log_base_dir}")

	s = requests.Session()
	sid = None
	while True:
		try:
			sid = login(s)
			save_log(f"{NAS} login success, sid:{sid[:4]}*****{sid[-4:]}")
	
			local_files_list = iter_local_files(local_log_base_dir, recursive=RECURSIVE)
			if not local_files_list:
				print("No files found under:", local_log_base_dir)
				return
			
			uploaded = 0
			skipped = 0
			failed = 0
	
			# 缓存：每个远端目录建立一次索引（仅该目录这一层）
			remote_dir_cache: Dict[str, Dict[str, Tuple[int, int]]] = {}
			for local_file in sorted(local_files_list):
				relative_file_path =  str(local_file.resolve().relative_to(local_log_base_dir.resolve()))
				remote_dir = remote_dir_for_file(REMOTE_FILE_PATH, relative_file_path)
				local_file_name = local_file.name
	
				do_upload = True
				if INCREMENTAL_UPDATE:
					if remote_dir not in remote_dir_cache:
						idx = build_remote_index(s, sid, remote_dir)
						remote_dir_cache[remote_dir] = idx
					do_upload = should_upload(local_file, remote_dir_cache[remote_dir], local_file_name)
	
				if not do_upload:
					#print(f"SKIP  {relative_file_path} (no change)")
					skipped += 1
					continue
	
				save_log(f"UPLOAD {relative_file_path} -> {remote_dir}/")
				resp = upload_file(s, sid, local_file, remote_dir)
	
				if resp.get("success"):
					uploaded += 1
					# 上传成功后，更新缓存索引，避免同目录重复 list
					try:
						st = local_file.stat()
						remote_dir_cache.setdefault(remote_dir, {})[local_file_name] = (int(st.st_size), int(time.time()))
					except Exception:
						pass
				else:
					failed += 1
					save_log(f"upload failed:{resp}")
	
			save_log(f"Done. uploaded={uploaded}, skipped={skipped}, failed={failed}")
	
		finally:
			if sid:
				logout(s, sid)
			time.sleep(WAIT_CYCLE)

if __name__ == "__main__":
	dsm_upload_files()