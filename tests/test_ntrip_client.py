import importlib.util
from pathlib import Path
import socket
import threading
import time
import unittest

spec = importlib.util.spec_from_file_location('rtk', Path(__file__).resolve().parents[1] / 'Quectel_RTK_Service/Quectel_RTK.py')
rtk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rtk)


def gga(utc='120000.00'):
    body = ('GNGGA,%s,4000.000,N,11600.000,E,1,12,1.0,40,M,0,M,,' % utc).encode()
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b'$' + body + ('*%02X\r\n' % checksum).encode()


class Serial:
    def __init__(self):
        self.pending = bytearray()
        self.written = bytearray()
        self.lock = threading.Lock()
        self.closed = False

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.pending)

    def feed(self, data):
        with self.lock:
            self.pending.extend(data)

    def read(self, count):
        with self.lock:
            data = bytes(self.pending[:count])
            del self.pending[:count]
        if not data:
            time.sleep(0.005)
        return data

    def write(self, data):
        # Deliberate short writes must not drop any correction bytes.
        n = min(7, len(data))
        self.written.extend(data[:n])
        return n

    def close(self):
        self.closed = True


class ClientTests(unittest.TestCase):
    def test_reader_drains_and_resynchronizes_and_rejects_stale(self):
        stream = Serial()
        client = rtk.NtripClient(stream=stream, gga_max_age=0.08)
        worker = threading.Thread(target=client.read_serial)
        worker.start()
        try:
            stream.feed(b''.join(gga('%06d.00' % i) for i in range(100)))
            stream.feed(b'$broken' + gga('120001.00'))
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                current = client.getGGABytes()
                if current and current[0] == gga('120001.00'):
                    break
                time.sleep(0.01)
            self.assertEqual(client.getGGABytes()[0], gga('120001.00'))
            stream.feed(gga('120002.00').replace(b'*', b'*00'))
            time.sleep(0.02)
            self.assertEqual(client.getGGABytes()[0], gga('120001.00'))
            time.sleep(0.1)
            self.assertIsNone(client.getGGABytes())
        finally:
            client.stop.set()
            worker.join(1)

    def exercise_server(self, response, expected, idle=False):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        received = bytearray()
        failures = []
        stream = Serial()
        stream.feed(gga())
        client = rtk.NtripClient(stream=stream, caster='127.0.0.1',
                                 port=listener.getsockname()[1], user='test:secret',
                                 gga_interval=0.05, gga_max_age=1, idle_timeout=0.1 if idle else 5)
        def server():
            try:
                with listener.accept()[0] as conn:
                    conn.settimeout(2)
                    request = b''
                    while b'\r\n\r\n' not in request:
                        request += conn.recv(4096)
                    for part in response:
                        conn.sendall(part)
                        time.sleep(0.01)
                    # No corrections during this interval: GGA must still flow.
                    end = time.monotonic() + (1.4 if idle else 0.35)
                    while time.monotonic() < end:
                        data = conn.recv(4096)
                        if not data:
                            break
                        received.extend(data)
            except Exception as exc:
                failures.append(exc)
            finally:
                listener.close()
        worker = threading.Thread(target=server)
        worker.start()
        with self.assertRaises((ConnectionError, TimeoutError)) as caught:
            client.readData()
        worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertFalse(failures, failures)
        self.assertGreaterEqual(received.count(b'$GNGGA,'), 3)
        self.assertEqual(bytes(stream.written), expected)
        self.assertTrue(stream.closed)
        return str(caught.exception)

    def test_icy_single_line_and_gga_during_correction_pause(self):
        payload = b'\xd3\x00\x01\xff\x00\x01\x02'
        reason = self.exercise_server([b'IC', b'Y 200 OK\r\n' + payload], payload)
        self.assertIn('Server EOF', reason)

    def test_fragmented_http_headers_preserve_binary_body(self):
        payload = b'\xd3\x00\x03\xff\x80\x00\x01\x02\x03'
        self.exercise_server([b'HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n', b'\r\n' + payload], payload)

    def test_no_corrections_timeout_keeps_sending_gga(self):
        reason = self.exercise_server([b'ICY 200 OK\r\n\r\n'], b'', idle=True)
        self.assertIn('No correction bytes', reason)


if __name__ == '__main__':
    unittest.main()
