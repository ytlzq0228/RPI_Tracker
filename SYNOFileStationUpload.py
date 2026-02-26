import os
import time
import json
import requests
import configparser
from requests.exceptions import RequestException
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
TIMEOUT_GET = 10
TIMEOUT_POST = 1800
WAIT_CYCLE = 5


MAX_RETRY_PER_FILE = 3
REMOTE_INDEX_TTL = 60  # 秒，避免每 5 秒都 list 远端

# ---------------- DSM WebAPI 帮助函数 ----------------
def api_get(session: requests.Session, url: str, params: Dict, verify) -> Dict:
	r = session.get(url, params=params, verify=verify, timeout=TIMEOUT_GET)
	r.raise_for_status()
	return r.json()

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
		save_log(f"[FILE COPY]logout:{j}")
	except Exception as e:
		save_log(f"[FILE COPY]logout error:{e}")

def upload_file(session: requests.Session, sid: str, local_file: Path, remote_dir: str) -> Dict:
	upload_url = f"{NAS}/webapi/entry.cgi"
	params = {
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
		r = session.post(upload_url, params=params, data=data, files=files, verify=VERIFY_TLS, timeout=TIMEOUT_POST)
		r.raise_for_status()
		try:
			return r.json()
		except Exception:
			return {"success": False, "raw": r.text}

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
	if recursive:
		return [p for p in base_dir.rglob("*") if p.is_file()]
	else:
		return [p for p in base_dir.iterdir() if p.is_file()]

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

def upload_with_retry(session, sid, local_file, remote_dir):
	last_exc = None
	for i in range(1, MAX_RETRY_PER_FILE + 1):
		try:
			return upload_file(session, sid, local_file, remote_dir)
		except RequestException as e:
			last_exc = e
			save_log(f"[FILE COPY]upload exception retry {i}/{MAX_RETRY_PER_FILE} file={local_file.name} err={e}")
			time.sleep(min(2 ** i, 30))
	raise last_exc

# ---------------- 主流程 ----------------
def dsm_upload_files():
	local_log_base_dir = Path(LOG_FILE_PATH)
	if not local_log_base_dir.exists() or not local_log_base_dir.is_dir():
		raise RuntimeError(f"LOG_FILE_PATH not a directory: {local_log_base_dir}")

	s = requests.Session()
	sid = login(s)
	save_log(f"[FILE COPY]{NAS} login success, sid:{sid[:4]}*****{sid[-4:]}")

	# 远端索引缓存：remote_dir -> (ts, idx)
	remote_dir_cache: Dict[str, Tuple[float, Dict[str, Tuple[int, int]]]] = {}

	while True:
		try:
			local_files_list = iter_local_files(local_log_base_dir, recursive=RECURSIVE)
			if not local_files_list:
				time.sleep(WAIT_CYCLE)
				continue

			uploaded = skipped = failed = 0

			for local_file in sorted(local_files_list):
				# 过滤无用文件
				if local_file.name == ".DS_Store":
					continue

				# Pi 环境：Path 原生相对路径 + posix
				rel = local_file.relative_to(local_log_base_dir)   # Path
				remote_dir = (Path(REMOTE_FILE_PATH) / rel.parent).as_posix()
				local_file_name = local_file.name

				# 增量：按 remote_dir 获取缓存（带 TTL）
				if INCREMENTAL_UPDATE:
					ts_idx = remote_dir_cache.get(remote_dir)
					if (ts_idx is None) or (time.time() - ts_idx[0] > REMOTE_INDEX_TTL):
						idx = build_remote_index(s, sid, remote_dir)
						remote_dir_cache[remote_dir] = (time.time(), idx)
					remote_idx = remote_dir_cache[remote_dir][1]

					if not should_upload(local_file, remote_idx, local_file_name):
						skipped += 1
						continue

				save_log(f"[FILE COPY]UPLOAD {rel.as_posix()} -> {remote_dir}/")

				# 先尝试上传（带异常重试）
				try:
					resp = upload_with_retry(s, sid, local_file, remote_dir)
				except Exception as e:
					failed += 1
					save_log(f"[FILE COPY]upload {rel.as_posix()} exception: {e}")
					continue

				# 如果返回 119：重登 + 立即重试一次
				if (not resp.get("success")) and resp.get("error", {}).get("code") == 119:
					save_log(f"[FILE COPY]SID invalid(119), relogin and retry once. old sid:{sid[:4]}*****{sid[-4:]}")
					sid = login(s)
					save_log(f"[FILE COPY]{NAS} relogin success, sid:{sid[:4]}*****{sid[-4:]}")

					try:
						resp = upload_with_retry(s, sid, local_file, remote_dir)
					except Exception as e:
						failed += 1
						save_log(f"[FILE COPY]upload after relogin failed {rel.as_posix()} exception: {e}")
						continue

				if resp.get("success"):
					uploaded += 1
					# 更新本目录缓存，避免下一次又传
					if INCREMENTAL_UPDATE:
						st = local_file.stat()
						ts, idx = remote_dir_cache.get(remote_dir, (time.time(), {}))
						idx[local_file_name] = (int(st.st_size), int(time.time()))
						remote_dir_cache[remote_dir] = (ts, idx)
				else:
					failed += 1
					save_log(f"[FILE COPY]upload {rel.as_posix()} failed resp:{resp}")

			save_log(f"[FILE COPY]Done. uploaded={uploaded}, skipped={skipped}, failed={failed}")

		except Exception as e:
			save_log(f"[FILE COPY]loop error {e}")

		finally:
			time.sleep(WAIT_CYCLE)

if __name__ == "__main__":
	dsm_upload_files()