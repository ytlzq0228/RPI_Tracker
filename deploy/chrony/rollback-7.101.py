#!/usr/bin/python3
"""Restore backed-up units/configuration and resume ntpd on 7.101."""
import json,shutil,subprocess,sys
from pathlib import Path
B=Path('/var/lib/rpi-tracker/time-alignment-20261118')
def run(*a): subprocess.run(a,check=True)
def writable():run('mount','-o','remount,rw','/')
if '--if-unconfirmed' in sys.argv and (B/'confirmed').exists():sys.exit(0)
try:
 run('systemctl','stop','chrony.service')
 for p,existed in json.loads((B/'state.json').read_text()).items():
  writable();q=Path(p)
  if existed:shutil.copy2(B/p.lstrip('/'),q)
  else:q.unlink(missing_ok=True)
 writable()
 Path('/etc/systemd/system/multi-user.target.wants/chrony.service').unlink(missing_ok=True)
 ntp=Path('/etc/systemd/system/ntp.service')
 if ntp.is_symlink() and str(ntp.readlink())=='/dev/null':ntp.unlink()
 link=Path('/etc/systemd/system/multi-user.target.wants/ntp.service')
 if not link.exists():link.symlink_to('/lib/systemd/system/ntp.service')
 run('systemctl','daemon-reload');run('systemctl','start','ntp.service')
finally:
 run('sync');run('mount','-o','remount,ro','/')
