import sqlite3
import json
import threading
from typing import Any, Dict, Optional

class PersistentQueue:
	"""
	Disk-backed FIFO queue with a deque-like subset API:
	  - append(item: dict)
	  - popleft() -> dict
	  - __len__() -> int
	  - __bool__() -> bool
	Backed by SQLite for crash/power-loss safety.
	"""

	def __init__(self, path: str, maxlen: int = 5000):
		self.path = path
		self.maxlen = maxlen
		self._lock = threading.Lock()
		self._init_db()

	def _connect(self) -> sqlite3.Connection:
		# 每次操作新建连接：线程更安全（producer/consumer 多线程场景）
		conn = sqlite3.connect(self.path, timeout=30, isolation_level=None)  # autocommit mode
		conn.execute("PRAGMA journal_mode=WAL;")
		conn.execute("PRAGMA synchronous=FULL;")  # 更稳：更抗断电；想要更快可改 NORMAL
		conn.execute("PRAGMA temp_store=MEMORY;")
		return conn

	def _init_db(self) -> None:
		with self._connect() as conn:
			conn.execute("""
				CREATE TABLE IF NOT EXISTS queue (
					id INTEGER PRIMARY KEY AUTOINCREMENT,
					item_json TEXT NOT NULL
				)
			""")
			conn.execute("CREATE INDEX IF NOT EXISTS idx_queue_id ON queue(id)")

	def __len__(self) -> int:
		with self._lock, self._connect() as conn:
			row = conn.execute("SELECT COUNT(*) FROM queue").fetchone()
			return int(row[0]) if row else 0

	def __bool__(self) -> bool:
		with self._lock, self._connect() as conn:
			row = conn.execute("SELECT 1 FROM queue LIMIT 1").fetchone()
			return row is not None

	def append(self, item: Dict[str, Any]) -> None:
		item_json = json.dumps(item, ensure_ascii=False, separators=(",", ":"))

		with self._lock, self._connect() as conn:
			conn.execute("BEGIN IMMEDIATE")
			try:
				conn.execute("INSERT INTO queue(item_json) VALUES (?)", (item_json,))
				# maxlen：超过上限则删除最老的多余条目（行为类似 deque(maxlen)）
				if self.maxlen is not None and self.maxlen != 0:
					conn.execute("""
						DELETE FROM queue
						WHERE id IN (
							SELECT id FROM queue
							ORDER BY id ASC
							LIMIT (SELECT MAX(COUNT(*) - ?, 0) FROM queue)
						)
					""", (int(self.maxlen),))
				conn.execute("COMMIT")
			except:
				conn.execute("ROLLBACK")
				raise

	def popleft(self) -> Dict[str, Any]:
		with self._lock, self._connect() as conn:
			conn.execute("BEGIN IMMEDIATE")
			try:
				row = conn.execute(
					"SELECT id, item_json FROM queue ORDER BY id ASC LIMIT 1"
				).fetchone()
				if row is None:
					conn.execute("ROLLBACK")
					raise IndexError("pop from an empty queue")

				row_id, item_json = row
				conn.execute("DELETE FROM queue WHERE id=?", (row_id,))
				conn.execute("COMMIT")
				return json.loads(item_json)
			except:
				try:
					conn.execute("ROLLBACK")
				except:
					pass
				raise

if __name__ == '__main__':
	SEND_QUEUE = PersistentQueue("send_queue.db", maxlen=0)
	SEND_QUEUE.append({"a":"1"})
	while len(SEND_QUEUE)>0:
		print(len(SEND_QUEUE))
		item = SEND_QUEUE.popleft()
		print(item)