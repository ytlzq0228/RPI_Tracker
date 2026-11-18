#!/usr/bin/python3
"""Restore the pre-migration gpsd service and standalone NTRIP client."""
import json
from pathlib import Path
import subprocess
import sys

BACKUP = Path('/var/lib/rpi-tracker/gpsd-rtcm3x-backup')
DROPIN = Path('/etc/systemd/system/gpsd.service.d/rtcm3x.conf')


def run(*args):
    subprocess.run(args, check=True)


def main():
    if '--if-unconfirmed' in sys.argv and (BACKUP / 'confirmed').exists():
        return
    metadata = json.loads((BACKUP / 'state.json').read_text())
    run('mount', '-o', 'remount,rw', '/')
    if metadata['dropin_existed']:
        DROPIN.write_bytes((BACKUP / 'rtcm3x.conf').read_bytes())
    else:
        DROPIN.unlink(missing_ok=True)
    run('systemctl', 'daemon-reload')
    run('systemctl', 'restart', 'gpsd.service')
    for unit in ('gpsd.service', 'gpsd.socket'):
        key = unit + '_enabled_before'
        if key in metadata:
            run('systemctl', 'enable' if metadata[key] else 'disable', unit)
    if metadata.get('gpsd_link_created'):
        link = Path('/dev/gpsd0')
        if link.is_symlink() and str(link.readlink()) == metadata['gpsd_link_target']:
            link.unlink()
    if metadata['rtk_enabled']:
        run('systemctl', 'enable', 'Quectel_RTK.service')
    if metadata['rtk_active']:
        run('systemctl', 'start', 'Quectel_RTK.service')
    holds = metadata.get('new_holds', [])
    if holds:
        run('apt-mark', 'unhold', *holds)
    if metadata['root_was_readonly']:
        run('sync')
        run('mount', '-o', 'remount,ro', '/')
    print('Original gpsd and NTRIP service restored.')


if __name__ == '__main__':
    main()
