#!/usr/bin/env python3
import time
import json
import math
import socket
import sys
from collections import deque
from argparse import ArgumentParser
import configparser
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException

import numpy as np
import gatt

# ---------------- 配置读取 ----------------
CONFIG_FILE = '/etc/GPS_config.ini'
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

# ----------------------------
# 可调参数（按车载常用）
# ----------------------------
SAMPLE_HZ = 50				  # 20ms 一次
WINDOW_SEC = 1.0
WINDOW_N = int(SAMPLE_HZ * WINDOW_SEC)

RING_SEC = 60				   # 事件回溯缓存长度
RING_N = int(SAMPLE_HZ * RING_SEC)

# 事件阈值（先给保守默认值，后面你可以按实测调）
TH_BRAKE_MPS2 = -3.0			# 急刹：纵向线加速度 < -3 m/s^2（持续判定另做）
TH_ACCEL_MPS2 = +2.5			# 急加：纵向线加速度 > +2.5 m/s^2
TH_TURN_MPS2 =  +3.0			# 急转：横向线加速度 |ay| > 3
TH_IMPACT_G = 2.5			   # 冲击：|a_total| 峰值 > 2.5g （如果你的单位是 m/s^2，会自动换算）
TH_BUMP_RMS_MPS2 = 2.0		  # 颠簸：垂向 RMS > 2 m/s^2（示例）

# “持续”判定（避免单点误报）
SUSTAIN_MS = 200				# 200ms
SUSTAIN_N = max(1, int(SAMPLE_HZ * (SUSTAIN_MS / 1000.0)))

# 坐标系映射：你的 aX/aY/aZ 不一定是车体 前/右/上
# 先按最常见假设：X=前进(纵向)，Y=右(横向)，Z=上(垂向)
AXIS_MAP = {
	"long": "aX",   # 纵向（加减速）
	"lat":  "aY",   # 横向（转弯）
	"vert": "aZ",   # 垂向（颠簸）
}

# 是否在事件里携带原始片段（会比较大；你也可以改成只存本地）
INCLUDE_RAW_SEGMENT_IN_EVENT = True
EVENT_PRE_SEC = 5
EVENT_POST_SEC = 10


# ----------------------------
# 解析器：把 BLE 帧解析成 dict
# ----------------------------
class ImuParser:
	# 缩放系数（沿用你现有代码）
	scaleAccel = 0.00478515625	  # 注：你注释写 9.8*16/32768，但这个数值更像 16/32768*9.8? 这里保持原值
	scaleQuat = 0.000030517578125
	scaleAngle = 0.0054931640625
	scaleAngleSpeed = 0.06103515625
	scaleMag = 0.15106201171875
	scaleTemperature = 0.01
	scaleAirPressure = 0.0002384185791
	scaleHeight = 0.0010728836

	@staticmethod
	def _i16(lo, hi) -> int:
		v = (hi << 8) | lo
		return np.int16(v).item()

	@staticmethod
	def _u32_le(b3, b4, b5, b6) -> int:
		return (b6 << 24) | (b5 << 16) | (b4 << 8) | b3

	@staticmethod
	def _i24_le(b0, b1, b2) -> int:
		u = (b2 << 16) | (b1 << 8) | b0
		if u & 0x800000:
			u |= 0xFF000000
		return int(u)					# now in [-8388608, 8388607]

	def parse(self, buf: bytes):
		# 返回 (ok, data_dict)
		if not buf or buf[0] != 0x11:
			return False, {}

		ctl = (buf[2] << 8) | buf[1]
		ms = self._u32_le(buf[3], buf[4], buf[5], buf[6])
		L = 7

		out = {
			"ctl": ctl,
			"ms": ms,
			"ts": time.time(),  # 本地接收时间戳（秒）
		}

		# 0x0001: aX aY aZ (去重力)
		if ctl & 0x0001:
			ax = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			ay = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			az = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			out.update({"aX": float(ax), "aY": float(ay), "aZ": float(az)})

		# 0x0002: AX AY AZ (含重力)
		if ctl & 0x0002:
			AX = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			AY = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			AZ = self._i16(buf[L], buf[L+1]) * self.scaleAccel; L += 2
			out.update({"AX": float(AX), "AY": float(AY), "AZ": float(AZ)})

		# 0x0004: GX GY GZ
		if ctl & 0x0004:
			gx = self._i16(buf[L], buf[L+1]) * self.scaleAngleSpeed; L += 2
			gy = self._i16(buf[L], buf[L+1]) * self.scaleAngleSpeed; L += 2
			gz = self._i16(buf[L], buf[L+1]) * self.scaleAngleSpeed; L += 2
			out.update({"GX": float(gx), "GY": float(gy), "GZ": float(gz)})

		# 0x0008: CX CY CZ
		if ctl & 0x0008:
			cx = self._i16(buf[L], buf[L+1]) * self.scaleMag; L += 2
			cy = self._i16(buf[L], buf[L+1]) * self.scaleMag; L += 2
			cz = self._i16(buf[L], buf[L+1]) * self.scaleMag; L += 2
			out.update({"CX": float(cx), "CY": float(cy), "CZ": float(cz)})

		# 0x0010: temp + airPressure(24) + height(24)
		if ctl & 0x0010:
			temp = self._i16(buf[L], buf[L+1]) * self.scaleTemperature; L += 2
			ap = self._i24_le(buf[L], buf[L+1], buf[L+2]) * self.scaleAirPressure; L += 3
			h = self._i24_le(buf[L], buf[L+1], buf[L+2]) * self.scaleHeight; L += 3
			out.update({"temperature": float(temp), "airPressure": float(ap), "height": float(h)})

		# 0x0020: quat wxyz
		if ctl & 0x0020:
			w = self._i16(buf[L], buf[L+1]) * self.scaleQuat; L += 2
			x = self._i16(buf[L], buf[L+1]) * self.scaleQuat; L += 2
			y = self._i16(buf[L], buf[L+1]) * self.scaleQuat; L += 2
			z = self._i16(buf[L], buf[L+1]) * self.scaleQuat; L += 2
			out.update({"w": float(w), "x": float(x), "y": float(y), "z": float(z)})

		# 0x0040: angleXYZ
		if ctl & 0x0040:
			roll = self._i16(buf[L], buf[L+1]) * self.scaleAngle; L += 2
			pitch = self._i16(buf[L], buf[L+1]) * self.scaleAngle; L += 2
			yaw = self._i16(buf[L], buf[L+1]) * self.scaleAngle; L += 2
			out.update({"angleX": float(roll), "angleY": float(pitch), "angleZ": float(yaw)})

		return True, out


# ----------------------------
# 指标聚合：1秒滑窗 + 事件检测 + 环形缓冲
# ----------------------------
class MetricsAggregator:
	def __init__(self):
		self.win = deque(maxlen=WINDOW_N)	 # 最近 1 秒样本（dict）
		self.ring = deque(maxlen=RING_N)	  # 最近 RING_SEC 秒原始样本（dict）
		self.last_emit_sec = None			 # 控制每秒输出一次
		self.last_a_long = None			   # jerk 计算
		self.sustain = {
			"brake": deque(maxlen=SUSTAIN_N),
			"accel": deque(maxlen=SUSTAIN_N),
			"turn": deque(maxlen=SUSTAIN_N),
			"impact": deque(maxlen=SUSTAIN_N),
			"bump": deque(maxlen=SUSTAIN_N),
		}
		self.pending_event = None
		self.pending_event_deadline = 0.0

	@staticmethod
	def _rms(vals):
		vals = np.asarray(vals, dtype=float)
		return float(np.sqrt(np.mean(vals * vals))) if len(vals) else 0.0

	@staticmethod
	def _std(vals):
		vals = np.asarray(vals, dtype=float)
		return float(np.std(vals)) if len(vals) else 0.0

	def _get_axis(self, sample: dict, key: str) -> float:
		# key: "long"/"lat"/"vert"
		field = AXIS_MAP[key]
		return float(sample.get(field, 0.0))

	def _a_total(self, sample: dict) -> float:
		# 优先用去重力 aX/aY/aZ，否则用 AX/AY/AZ
		if all(k in sample for k in ("aX", "aY", "aZ")):
			ax, ay, az = sample["aX"], sample["aY"], sample["aZ"]
		elif all(k in sample for k in ("AX", "AY", "AZ")):
			ax, ay, az = sample["AX"], sample["AY"], sample["AZ"]
		else:
			return 0.0
		return float(math.sqrt(ax*ax + ay*ay + az*az))

	def add_sample(self, sample: dict):
		# sample 必须有 ts
		self.win.append(sample)
		self.ring.append(sample)

		now = sample["ts"]
		sec = int(now)

		# ---- 事件检测（在线）----
		a_long = self._get_axis(sample, "long")
		a_lat = self._get_axis(sample, "lat")
		a_vert = self._get_axis(sample, "vert")

		# jerk（差分 / dt）
		jerk = 0.0
		if self.last_a_long is not None:
			jerk = (a_long - self.last_a_long) * SAMPLE_HZ
		self.last_a_long = a_long

		# 单位兼容：如果你的加速度单位是 g，这里换算一下阈值
		# 简单判断：如果静止时 AZ/|AXYZ| 大约 9~10，则单位可能是 m/s^2；若大约 1，则单位可能是 g
		# 这里用 a_total 的量级估算
		a_tot = self._a_total(sample)
		looks_like_g = (0.2 < a_tot < 3.0)   # 粗判：接近 1 的量级
		impact_thr = TH_IMPACT_G if looks_like_g else (TH_IMPACT_G * 9.80665)

		# sustain 采样
		self.sustain["brake"].append(1 if a_long < TH_BRAKE_MPS2 else 0)
		self.sustain["accel"].append(1 if a_long > TH_ACCEL_MPS2 else 0)
		self.sustain["turn"].append(1 if abs(a_lat) > TH_TURN_MPS2 else 0)
		self.sustain["impact"].append(1 if a_tot > impact_thr else 0)
		# bump 用垂向绝对值的瞬时门限做粗判；更稳的是用 1秒 RMS 判定（在 emit 时做）
		self.sustain["bump"].append(1 if abs(a_vert) > (TH_BUMP_RMS_MPS2 * 2) else 0)

		def sustained(name: str) -> bool:
			d = self.sustain[name]
			return len(d) == d.maxlen and sum(d) == d.maxlen

		# 如果触发事件，先记一个 pending（等 post 窗口采满）
		if self.pending_event is None:
			if sustained("impact"):
				self._start_event("impact", sample)
			elif sustained("brake"):
				self._start_event("harsh_brake", sample)
			elif sustained("accel"):
				self._start_event("harsh_accel", sample)
			elif sustained("turn"):
				self._start_event("sharp_turn", sample)

		# 如果已有 pending_event，到时间了就输出（含 pre/post 片段）
		if self.pending_event is not None and now >= self.pending_event_deadline:
			ev = self._finalize_event()
			if ev is not None:
				print(json.dumps({"type": "event", **ev}, ensure_ascii=False))

		# ---- 每秒输出一次指标 ----
		if self.last_emit_sec is None:
			self.last_emit_sec = sec

		if sec != self.last_emit_sec and len(self.win) >= max(10, WINDOW_N // 2):
			metrics = self.compute_metrics()
			print(json.dumps({"type": "metrics_1s", **metrics}, ensure_ascii=False))
			print(metrics.get("bump_index"))
			self.last_emit_sec = sec

	def _start_event(self, ev_type: str, sample: dict):
		self.pending_event = {
			"eventType": ev_type,
			"eventTs": sample["ts"],
			"ms": sample.get("ms"),
			"snapshot": {
				"aX": sample.get("aX"), "aY": sample.get("aY"), "aZ": sample.get("aZ"),
				"AX": sample.get("AX"), "AY": sample.get("AY"), "AZ": sample.get("AZ"),
				"GX": sample.get("GX"), "GY": sample.get("GY"), "GZ": sample.get("GZ"),
				"angleX": sample.get("angleX"), "angleY": sample.get("angleY"), "angleZ": sample.get("angleZ"),
			},
		}
		self.pending_event_deadline = sample["ts"] + EVENT_POST_SEC

	def _finalize_event(self):
		if self.pending_event is None:
			return None

		t0 = self.pending_event["eventTs"]
		t1 = t0 - EVENT_PRE_SEC
		t2 = t0 + EVENT_POST_SEC

		# 从 ring 里切片
		seg = [s for s in self.ring if (s["ts"] >= t1 and s["ts"] <= t2)]

		# 事件统计
		a_long = [self._get_axis(s, "long") for s in seg]
		a_lat = [self._get_axis(s, "lat") for s in seg]
		a_vert = [self._get_axis(s, "vert") for s in seg]
		a_tot = [self._a_total(s) for s in seg]

		ev = dict(self.pending_event)
		ev.update({
			"preSec": EVENT_PRE_SEC,
			"postSec": EVENT_POST_SEC,
			"samples": len(seg),
			"peakLong": float(np.min(a_long) if a_long else 0.0),  # 急刹更关注最小
			"peakAccel": float(np.max(a_long) if a_long else 0.0),
			"peakLat": float(np.max(np.abs(a_lat)) if a_lat else 0.0),
			"peakVert": float(np.max(np.abs(a_vert)) if a_vert else 0.0),
			"peakTotal": float(np.max(a_tot) if a_tot else 0.0),
		})

		if INCLUDE_RAW_SEGMENT_IN_EVENT:
			# 为了避免太大：只保留关键字段 + 可选降采样
			decim = 1  # 你可以改成 2/5/10 做降采样
			raw = []
			for i, s in enumerate(seg):
				if i % decim != 0:
					continue
				raw.append({
					"t": round(s["ts"] - t0, 3),  # 相对事件时刻
					"aX": s.get("aX"), "aY": s.get("aY"), "aZ": s.get("aZ"),
					"GX": s.get("GX"), "GY": s.get("GY"), "GZ": s.get("GZ"),
					"angleX": s.get("angleX"), "angleY": s.get("angleY"), "angleZ": s.get("angleZ"),
				})
			ev["raw"] = raw

		# 清空 pending
		self.pending_event = None
		self.pending_event_deadline = 0.0
		return ev

	def compute_metrics(self) -> dict:
		# 在 1秒窗口内计算指标（以去重力 a* 优先）
		samples = list(self.win)
		if not samples:
			return {}

		ts_end = samples[-1]["ts"]
		ts_start = samples[0]["ts"]

		long_vals = [self._get_axis(s, "long") for s in samples]
		lat_vals = [self._get_axis(s, "lat") for s in samples]
		vert_vals = [self._get_axis(s, "vert") for s in samples]

		# jerk（纵向）
		jerks = []
		prev = None
		for v in long_vals:
			if prev is not None:
				jerks.append((v - prev) * SAMPLE_HZ)
			prev = v

		# 角速度（用于转向/稳定性）
		gz_vals = [float(s.get("GZ", 0.0)) for s in samples]

		# 姿态（如果有）
		roll = float(samples[-1].get("angleX", 0.0))
		pitch = float(samples[-1].get("angleY", 0.0))
		yaw = float(samples[-1].get("angleZ", 0.0))

		# 颠簸指数：先用垂向 RMS
		bump_index = self._rms(vert_vals)

		# 事件 flags（这 1 秒内是否出现过）
		harsh_brake = (np.min(long_vals) < TH_BRAKE_MPS2) if long_vals else False
		harsh_accel = (np.max(long_vals) > TH_ACCEL_MPS2) if long_vals else False
		sharp_turn = (np.max(np.abs(lat_vals)) > TH_TURN_MPS2) if lat_vals else False

		# 简单的“平顺性评分”示例（0~100）
		# 你后续可以把权重/阈值按实测调优
		jerk_rms = self._rms(jerks)
		score = 100.0
		score -= min(40.0, max(0.0, (abs(np.min(long_vals)) - 2.0) * 10.0))  # 急刹惩罚
		score -= min(30.0, max(0.0, (np.max(long_vals) - 2.0) * 8.0))		# 急加惩罚
		score -= min(20.0, max(0.0, (np.max(np.abs(lat_vals)) - 2.5) * 6.0))  # 急转惩罚
		score -= min(20.0, max(0.0, (bump_index - 1.5) * 8.0))			   # 颠簸惩罚
		score -= min(20.0, max(0.0, (jerk_rms - 5.0) * 2.0))				 # 不平顺惩罚
		score = float(max(0.0, min(100.0, score)))

		return {
			"tsStart": ts_start,
			"tsEnd": ts_end,
			"n": len(samples),

			# 纵向/横向/垂向（1秒统计）
			"aLong_mean": float(np.mean(long_vals)),
			"aLong_min": float(np.min(long_vals)),
			"aLong_max": float(np.max(long_vals)),
			"aLong_rms": self._rms(long_vals),
			"aLong_std": self._std(long_vals),

			"aLat_mean": float(np.mean(lat_vals)),
			"aLat_min": float(np.min(lat_vals)),
			"aLat_max": float(np.max(lat_vals)),
			"aLat_rms": self._rms(lat_vals),
			"aLat_std": self._std(lat_vals),

			"aVert_mean": float(np.mean(vert_vals)),
			"aVert_min": float(np.min(vert_vals)),
			"aVert_max": float(np.max(vert_vals)),
			"aVert_rms": self._rms(vert_vals),
			"aVert_std": self._std(vert_vals),

			# jerk / yaw rate
			"jerk_rms": jerk_rms,
			"jerk_max": float(np.max(np.abs(jerks)) if jerks else 0.0),

			"gz_rms": self._rms(gz_vals),
			"gz_max": float(np.max(np.abs(gz_vals)) if gz_vals else 0.0),

			# 姿态（取末样本；你也可以改成均值/中位数）
			"roll": roll,
			"pitch": pitch,
			"yaw": yaw,

			# 路面颠簸指数（当前用 RMS）
			"bump_index": bump_index,

			# flags + score
			"flag_harsh_brake": bool(harsh_brake),
			"flag_harsh_accel": bool(harsh_accel),
			"flag_sharp_turn": bool(sharp_turn),
			"score_1s": score,
		}


# ----------------------------
# BLE 设备：连接 + 订阅 + 接收
# ----------------------------
class AnyDevice(gatt.Device):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.sock_pc = None
		self.parse_local = True

		self.parser = ImuParser()
		self.agg = MetricsAggregator()

	def connect_succeeded(self):
		super().connect_succeeded()
		print(f"[{self.mac_address}] Connected")

	def connect_failed(self, error):
		super().connect_failed(error)
		print(f"[{self.mac_address}] Connection failed: {error}")

	def disconnect_succeeded(self):
		super().disconnect_succeeded()
		print(f"[{self.mac_address}] Disconnected")

	def services_resolved(self):
		super().services_resolved()
		print(f"[{self.mac_address}] Resolved services")

		# 建立 uuid->characteristic 映射（修复你原来的 service 取最后一个的问题）
		ch_map = {}
		for s in self.services:
			for c in s.characteristics:
				ch_map[c.uuid.lower()] = c

		for s in self.services:
			print(f"[{self.mac_address}]\tService [{s.uuid}]")
			for c in s.characteristics:
				print(f"[{self.mac_address}]\t\tCharacteristic [{c.uuid}]")

		uuid_ae01 = "0000ae01-0000-1000-8000-00805f9b34fb"
		uuid_ae02 = "0000ae02-0000-1000-8000-00805f9b34fb"

		if uuid_ae01 not in ch_map or uuid_ae02 not in ch_map:
			print(f"[{self.mac_address}] ERROR: AE01/AE02 characteristic not found")
			return

		ctrl = ch_map[uuid_ae01]
		data = ch_map[uuid_ae02]

		# 设备保持连接/高速
		ctrl.write_value(bytes([0x29]))
		time.sleep(0.05)
		ctrl.write_value(bytes([0x46]))
		time.sleep(0.05)

		# 参数设置
		isCompassOn = 0
		barometerFilter = 2
		Cmd_ReportTag = 0x7F

		params = bytearray(11)
		params[0] = 0x12
		params[1] = 5
		params[2] = 255
		params[3] = 0
		params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
		params[5] = 50  # 建议和实际 20ms一致：50Hz；你之前写 60Hz 也可以，但要确保设备真输出 60Hz
		params[6] = 1
		params[7] = 3
		params[8] = 5
		params[9] = Cmd_ReportTag & 0xFF
		params[10] = (Cmd_ReportTag >> 8) & 0xFF
		ctrl.write_value(params)
		time.sleep(0.05)

		# 开始主动上报
		ctrl.write_value(bytes([0x19]))
		time.sleep(0.05)

		# 启用通知
		data.enable_notifications()

	def characteristic_value_updated(self, characteristic, value):
		# 原始字节打印（你调试时可开）
		# print("AE02:", value.hex(), "len=", len(value))

		ok, sample = self.parser.parse(value)
		if not ok:
			return

		# 1) 本地指标加工（每秒输出一条 metrics_1s；事件输出 type=event）
		self.agg.add_sample(sample)

		# 2) 转发原始 BLE 帧（可选）
		if self.sock_pc is not None:
			# 注意：TCP 是流，建议加长度前缀；这里保持与你原代码一致
			self.sock_pc.sendall(value)



def main_start():
	arg_parser = ArgumentParser(description="BLE IMU Metrics Processor")
	arg_parser.add_argument("mac_address", help="MAC address of IMU")
	arg_parser.add_argument("host_ip", nargs="?", default=None, help="Optional TCP host to forward raw frames to (port 6666)")
	args = arg_parser.parse_args()

	sock = None
	if args.host_ip:
		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sock.connect((args.host_ip, 6666))
		except Exception as e:
			print("Could not connect to server:", e)
			sys.exit(1)

	print("Connecting bluetooth ...")
	manager = gatt.DeviceManager(adapter_name="hci0")
	device = AnyDevice(manager=manager, mac_address=args.mac_address)
	device.sock_pc = sock
	device.connect()
	manager.run()


import threading
import queue
import time
from typing import Optional, Dict, Any

import gatt


class BleImuWorker:
	def __init__(self, mac: str, adapter: str = "hci0", host_ip: Optional[str] = None):
		self.mac = mac
		self.adapter = adapter
		self.host_ip = host_ip

		self.thread: Optional[threading.Thread] = None
		self.stop_event = threading.Event()

		self.metrics_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=3000)
		self.event_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1000)

		self.latest_metrics: Optional[Dict[str, Any]] = None
		self.latest_event: Optional[Dict[str, Any]] = None
		self._lock = threading.Lock()

		self.manager: Optional[gatt.DeviceManager] = None
		self.device = None  # AnyDevice

	def start(self):
		if self.thread and self.thread.is_alive():
			return
		self.stop_event.clear()
		self.thread = threading.Thread(target=self._run, name="ble-imu-worker", daemon=True)
		self.thread.start()

	def stop(self, timeout: float = 5.0):
		self.stop_event.set()

		# 关键：让 gatt 的 GLib loop 退出
		try:
			if self.manager is not None:
				# gatt_linux.DeviceManager 通常有 stop()，没有也没关系，except 掉
				self.manager.stop()
		except Exception:
			pass

		if self.thread:
			self.thread.join(timeout=timeout)

	def _run(self):
		# 这里延迟一点，避免 FastAPI 启动阶段资源竞争
		time.sleep(0.2)

		# 1) 构造 processor，回调里回传给主程序
		def on_metrics(m: Dict[str, Any]):
			with self._lock:
				self.latest_metrics = m
			# 队列满了就丢最老的，保证线程不阻塞
			try:
				self.metrics_q.put_nowait(m)
			except queue.Full:
				try:
					_ = self.metrics_q.get_nowait()
				except queue.Empty:
					pass
				try:
					self.metrics_q.put_nowait(m)
				except queue.Full:
					pass

		def on_event(e: Dict[str, Any]):
			with self._lock:
				self.latest_event = e
			try:
				self.event_q.put_nowait(e)
			except queue.Full:
				try:
					_ = self.event_q.get_nowait()
				except queue.Empty:
					pass
				try:
					self.event_q.put_nowait(e)
				except queue.Full:
					pass

		processor = MetricsProcessor(
			sample_rate_hz=50.0,
			window_sec=1.0,
			raw_buffer_sec=60.0,
			event_pre_sec=5.0,
			event_post_sec=10.0,
			on_metrics=on_metrics,
			on_event=on_event,
		)

		# 2) 启动 gatt 主循环（阻塞）
		self.manager = gatt.DeviceManager(adapter_name=self.adapter)

		# 你的 AnyDevice 需要接收 processor 的话，用 kwargs 传进去
		# 如果你当前 AnyDevice 构造不需要 processor，就把 processor 注入到 device 上也行
		self.device = AnyDevice(manager=self.manager, mac_address=self.mac, processor=processor)

		self.device.connect()

		# 3) 关键点：退出条件
		# gatt 的 run() 会阻塞；stop() 会让 loop 退出
		# 所以这里简单 run()，由 stop() 来终止
		try:
			self.manager.run()
		except Exception as e:
			# 你可以在这里记录日志
			# print("BLE worker crashed:", e)
			pass

BLE_MAC = "56:D8:18:8E:D2:FC"

worker: BleImuWorker | None = None

@asynccontextmanager
async def lifespan(app: FastAPI):
	global worker

	# 避免 uvicorn --reload 双进程重复启动：只在主进程启动
	# uvicorn 会设置 RUN_MAIN 或者你也可以用更严格的方式识别
	# 如果你不用 reload，这段可以不要
	import os
	if os.environ.get("RUN_MAIN") == "true" or os.environ.get("RUN_MAIN") is None:
		worker = BleImuWorker(mac=BLE_MAC, adapter="hci0")
		worker.start()

	yield

	if worker:
		worker.stop(timeout=5.0)

app = FastAPI(lifespan=lifespan)

@app.get("/imu/latest")
def imu_latest():
	if not worker:
		raise HTTPException(503, "worker not started")
	with worker._lock:
		return worker.latest_metrics or {"type": "metrics_1s", "status": "no_data"}

@app.get("/imu/poll")
def imu_poll(n: int = 10):
	if not worker:
		raise HTTPException(503, "worker not started")
	out = []
	for _ in range(max(0, min(n, 200))):
		try:
			out.append(worker.metrics_q.get_nowait())
		except Exception:
			break
	return {"n": len(out), "items": out}




if __name__ == "__main__":
	import logging
	import uvicorn
	logging.getLogger("uvicorn.access").setLevel(logging.ERROR)
	uvicorn.run(app, host="0.0.0.0", port=5053)