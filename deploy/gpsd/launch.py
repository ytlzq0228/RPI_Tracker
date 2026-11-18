#!/usr/bin/python3
"""Launch the patched gpsd with the project's existing NTRIP configuration."""
import configparser
import os
from pathlib import Path

BINARY = '/opt/gpsd-3.22-rtcm3x/sbin/gpsd'
CONFIG = '/etc/GPS_config.ini'


def main():
    config = configparser.ConfigParser(interpolation=None)
    if not config.read(CONFIG):
        raise SystemExit('GPS configuration file is missing')
    # Passive mode allows RTCM writes but avoids receiver reconfiguration.
    # gpsd's standard /dev/gpsd0 alias associates this serial source with
    # /dev/pps0, retaining GPS/PPS SHM slots even when using the USB port.
    device = os.environ.get('GPSD_SERIAL_DEVICE', '/dev/gpsd0')
    if device == '/dev/gpsd0':
        target = config.get('RTK_CONFIG', 'rtk_port', fallback='/dev/ttyUSB0').strip()
        if not Path(target).is_char_device():
            raise SystemExit('Configured GNSS serial device is not ready')
        link = Path(device)
        if link.is_symlink():
            if os.readlink(link) != target:
                raise SystemExit('/dev/gpsd0 already points to a different device')
        elif link.exists():
            raise SystemExit('/dev/gpsd0 is already in use')
        else:
            link.symlink_to(target)
    args = [BINARY, '-N', '-n', '-G', '-p', '-s', '115200', device]
    if config.getboolean('RTK_CONFIG', 'enable', fallback=False):
        rtk = config['RTK_CONFIG']
        user, password = rtk['username'], rtk['password']
        host, port = rtk['ntrip_server'].strip(), rtk.getint('ntrip_port')
        mount = rtk['mountpoint'].strip().lstrip('/')
        if not (user and password and host and mount and 0 < port < 65536):
            raise SystemExit('Incomplete NTRIP configuration')
        args.append(f'ntrip://{user}:{password}@{host}:{port}/{mount}')
    os.execv(BINARY, args)


if __name__ == '__main__':
    main()
