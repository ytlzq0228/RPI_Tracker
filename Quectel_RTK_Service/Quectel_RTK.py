#!/usr/bin/env -S python3 -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

"""

import socket
import sys
import base64
import time
import threading
import ssl as tls
from optparse import OptionParser

import serial

version = 0.3
useragent = 'NTRIP RPITracker/%.1f' % version


class NtripClient:
    """One serial reader, one periodic GGA sender, one RTCM receive/forward loop."""

    def __init__(self, buffer=4096, user='', out=None, port=2101, caster='',
                 mountpoint='', host=False, lat=0, lon=0, height=0, ssl=False,
                 verbose=False, UDP_Port=None, V2=False, headerFile=sys.stderr,
                 headerOutput=False, maxConnectTime=0, serial_port='/dev/ttyUSB0',
                 maxReconnect=1, gga_interval=5, gga_max_age=10, idle_timeout=60,
                 stream=None):
        self.buffer = buffer
        self.user = base64.b64encode(user.encode()).decode()
        self.port, self.caster = port, caster
        self.mountpoint = '/' + mountpoint.lstrip('/')
        self.ssl, self.V2, self.verbose = ssl, V2, verbose
        self.headerFile, self.headerOutput = headerFile, headerOutput
        self.maxConnectTime = maxConnectTime or 0
        self.maxReconnect = maxReconnect
        self.gga_interval, self.gga_max_age = gga_interval, gga_max_age
        self.idle_timeout = idle_timeout
        self.out = out
        self.stream = stream if stream is not None else serial.Serial(
            serial_port, 115200, timeout=0.2, write_timeout=3)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest_gga = None
        self.serial_error = None
        self.socket = None
        self.UDP_Port = UDP_Port
        self.UDP_socket = None
        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def log(self, text):
        print(text, file=sys.stderr, flush=True)

    @staticmethod
    def valid_gga(line):
        if not line.startswith((b'$GNGGA,', b'$GPGGA,')):
            return False
        try:
            body, checksum = line[1:].split(b'*')
            value = 0
            for byte in body:
                value ^= byte
            fields = body.split(b',')
            return (len(checksum) == 2 and value == int(checksum, 16)
                    and len(fields) >= 15 and int(fields[6]) > 0
                    and bool(fields[1]) and bool(fields[2]) and bool(fields[4]))
        except (ValueError, IndexError):
            return False

    def read_serial(self):
        pending = b''
        try:
            while not self.stop.is_set():
                chunk = self.stream.read(min(max(self.stream.in_waiting, 1), 4096))
                if not chunk:
                    continue
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    # Recover at a fresh sentence after a damaged/truncated sentence.
                    line = line[line.rfind(b'$'):].strip() if b'$' in line else b''
                    if self.valid_gga(line):
                        with self.lock:
                            self.latest_gga = (line + b'\r\n', time.monotonic())
                if len(pending) > 8192:
                    pending = b''
        except Exception as exc:
            self.serial_error = exc
            self.stop.set()

    def getGGABytes(self):
        with self.lock:
            snapshot = self.latest_gga
        if snapshot and time.monotonic() - snapshot[1] <= self.gga_max_age:
            return snapshot
        return None

    def getMountPointBytes(self):
        request = ('GET %s HTTP/1.0\r\nHost: %s:%s\r\nUser-Agent: %s\r\n'
                   'Authorization: Basic %s\r\n' %
                   (self.mountpoint, self.caster, self.port, useragent, self.user))
        if self.V2:
            request += 'Ntrip-Version: Ntrip/2.0\r\n'
        return (request + '\r\n').encode('ascii')

    def read_response(self, sock):
        data = b''
        deadline = time.monotonic() + 10
        while b'\r\n' not in data:
            if time.monotonic() > deadline:
                raise TimeoutError('NTRIP status timeout')
            part = sock.recv(4096)
            if not part:
                raise ConnectionError('Server EOF before NTRIP status')
            data += part
            if len(data) > 16384:
                raise ConnectionError('NTRIP response headers too large')
        status, rest = data.split(b'\r\n', 1)
        self.log('NTRIP status: ' + status.decode('ascii', 'replace'))
        if status not in (b'ICY 200 OK', b'HTTP/1.0 200 OK', b'HTTP/1.1 200 OK'):
            raise ConnectionError('NTRIP request rejected')
        if status.startswith(b'ICY'):
            return rest
        while b'\r\n\r\n' not in data:
            if len(data) > 16384 or time.monotonic() > deadline:
                raise ConnectionError('Incomplete/oversized HTTP headers')
            part = sock.recv(4096)
            if not part:
                raise ConnectionError('Server EOF in HTTP headers')
            data += part
        headers, body = data.split(b'\r\n\r\n', 1)
        if b'transfer-encoding:' in headers.lower():
            raise ConnectionError('Unsupported HTTP transfer encoding; use NTRIP v1')
        if self.headerOutput:
            self.headerFile.write(headers.decode('ascii', 'replace') + '\n')
        return body

    def send_gga(self, sock, ended, errors):
        next_send = 0
        warned = False
        try:
            while not ended.is_set() and not self.stop.is_set():
                now = time.monotonic()
                if now >= next_send:
                    snapshot = self.getGGABytes()
                    if snapshot:
                        sentence, received = snapshot
                        sock.sendall(sentence)
                        fields = sentence.split(b',')
                        if self.verbose:
                            self.log('GGA sent: utc=%s age=%.2fs quality=%s' %
                                     (fields[1].decode(), now - received, fields[6].decode()))
                        next_send = now + self.gga_interval
                        warned = False
                    elif not warned:
                        self.log('Waiting for fresh valid serial GGA; stale GGA not sent')
                        warned = True
                ended.wait(min(0.2, self.gga_interval))
        except Exception as exc:
            errors.append(exc)
            ended.set()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def forward(self, data):
        view = memoryview(data)
        while view:
            n = self.stream.write(view)
            if not n:
                raise IOError('Serial write made no progress')
            view = view[n:]
        if self.out:
            self.out.write(data)
        if self.UDP_socket:
            self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))

    def run_connection(self):
        sock = socket.create_connection((self.caster, self.port), timeout=10)
        self.socket = sock
        ended, errors = threading.Event(), []
        sender = None
        try:
            if self.ssl:
                sock = tls.create_default_context().wrap_socket(sock, server_hostname=self.caster)
                self.socket = sock
            sock.sendall(self.getMountPointBytes())
            first = self.read_response(sock)
            sock.settimeout(1)
            sender = threading.Thread(target=self.send_gga, args=(sock, ended, errors), daemon=True)
            sender.start()
            start = last_data = last_report = time.monotonic()
            total = 0
            leading = True
            while not self.stop.is_set():
                if errors:
                    raise ConnectionError('GGA send failed: %s' % errors[0])
                now = time.monotonic()
                if self.maxConnectTime and now - start >= self.maxConnectTime:
                    return
                try:
                    data = first if first else sock.recv(self.buffer)
                    first = b''
                except socket.timeout:
                    if time.monotonic() - last_data >= self.idle_timeout:
                        raise TimeoutError('No correction bytes for %.0fs (GGA sender remained active)' % self.idle_timeout)
                    continue
                if not data:
                    raise ConnectionError('Server EOF: connection closed by peer')
                if leading:
                    data = data.lstrip(b'\r\n')  # Optional blank line after ICY status.
                    if not data:
                        continue
                    leading = False
                self.forward(data)
                last_data = time.monotonic()
                total += len(data)
                if last_data - last_report >= 15:
                    self.log('RTCM forwarded: bytes=%d connection_age=%.0fs' % (total, last_data - start))
                    last_report = last_data
            if self.serial_error:
                raise IOError('Serial reader failed: %s' % self.serial_error)
        finally:
            ended.set()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            if sender:
                sender.join(timeout=3)
            sock.close()
            self.socket = None

    def readData(self):
        reader = threading.Thread(target=self.read_serial, daemon=True)
        reader.start()
        try:
            for attempt in range(max(1, self.maxReconnect)):
                try:
                    self.run_connection()
                    return
                except (OSError, serial.SerialException) as exc:
                    self.log('Connection ended: %s' % exc)
                    if self.serial_error or attempt + 1 >= max(1, self.maxReconnect):
                        raise
                    self.stop.wait(min(2 ** attempt, 30))
        finally:
            self.stop.set()
            reader.join(timeout=3)
            self.stream.close()
            if self.UDP_socket:
                self.UDP_socket.close()

if __name__ == '__main__':
    usage="NtripClient.py [options] [caster] [port] mountpoint"
    parser=OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS", help="The Ntripcaster username.  Default: %default")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS", help="The Ntripcaster password. Default: %default")
    parser.add_option("-o", "--org", type="string", dest="org", help="Use IBSS and the provided organization for the user. Caster and Port are not needed in this case Default: %default")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg", help="The org that the base is in. IBSS Only, assumed to be the user org")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09, help="Your latitude.  Default: %default")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66, help="Your longitude.  Default: %default")
    parser.add_option("-e", "--height", type="float", dest="height", default=1200, help="Your ellipsoid height.  Default: %default")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=True, help="Verbose")
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False, help="Use SSL for the connection")
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False, help="Include host header, should be on for IBSS")
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1, help="Number of reconnections")
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None, help="Broadcast recieved data on the provided port")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False, help="Make a NTRIP V2 Connection")
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None, help="Write to this file, instead of stdout")
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=None, help="Maximum length of the connection, in seconds")
    parser.add_option("-P", "--port", type="string", dest="serial_port", default="/dev/ttyUSB0", help="serial_port default /dev/ttyUSB0")

    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False, help="Write headers to stderr")
    parser.add_option("--HeaderFile", type="string", dest="headerFile", default=None, help="Write headers to this file, instead of stderr.")
    (options, args) = parser.parse_args()
    # print(args)
    # print(len(args))
    ntripArgs = {}
    ntripArgs['lat']=options.lat
    ntripArgs['lon']=options.lon
    ntripArgs['height']=options.height
    ntripArgs['host']=options.host

    if options.ssl:
        import ssl
        ntripArgs['ssl']=True
    else:
        ntripArgs['ssl']=False


#ignore
    if options.org:
        if len(args) != 1 :
            print ("Incorrect number of arguments for IBSS. You do not need to provide the server and port\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user']=options.user+"."+options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster']=options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster']=options.org + ".ibss.trimbleos.com"
        if options.ssl :
            ntripArgs['port']=52101
        else :
            ntripArgs['port']=2101
        ntripArgs['mountpoint']=args[0]

    else:
        if len(args) != 3 :
            print ("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
# execuet
        ntripArgs['user']=options.user+":"+options.password
        ntripArgs['caster']=args[0]
        ntripArgs['port']=int(args[1])
        ntripArgs['mountpoint']=args[2]

    if ntripArgs['mountpoint'][0:1] !="/":
        ntripArgs['mountpoint'] = "/"+ntripArgs['mountpoint']

    ntripArgs['V2']=options.V2

    ntripArgs['verbose']=options.verbose
    ntripArgs['headerOutput']=options.headerOutput

    if options.UDP:
         ntripArgs['UDP_Port']=int(options.UDP)

    ntripArgs['maxReconnect'] = options.maxReconnect
    ntripArgs['maxConnectTime'] = options.maxConnectTime
    ntripArgs['serial_port'] = options.serial_port

    if options.verbose:
        print ("Server: " + ntripArgs['caster'])
        print ("Port: " + str(ntripArgs['port']))
        print ("Authentication: configured")
        print ("mountpoint: " +ntripArgs['mountpoint'])
        print ("Reconnects: " + str(options.maxReconnect))
        print ("Max Connect Time: " + str(options.maxConnectTime))
        if ntripArgs['V2']:
            print ("NTRIP: V2")
        else:
            print ("NTRIP: V1")
        if ntripArgs["ssl"]:
            print ("SSL Connection")
        else:
            print ("Uncrypted Connection")
        print ("")



    fileOutput=False

    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out']=f
        fileOutput=True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile']=h
        ntripArgs['headerOutput']=True

    n = NtripClient(**ntripArgs)
    try:
        n.readData()
    except KeyboardInterrupt:
        pass
    except (OSError, serial.SerialException):
        sys.exit(1)
    finally:
        if fileOutput:
            f.close()
        if options.headerFile:
            h.close()
