import subprocess,shutil,json,os
from pathlib import Path
B=Path('/var/lib/rpi-tracker/time-alignment-20261118');S=Path('/tmp/align-time7');D=Path('/tmp/chrony-config')
def run(*a):return subprocess.run(a,check=True,capture_output=True,text=True).stdout
paths=['/etc/systemd/system/RPI_Tracker.service','/etc/systemd/system/Quectel_RTK.service','/etc/systemd/system/rtc-sync.service','/etc/systemd/system/chrony.service','/etc/chrony/chrony.conf']
if B.exists():raise SystemExit('Backup exists, inspect before retry')
run('mount','-o','remount,rw','/')
try:
 B.mkdir(parents=True)
 state={}
 for p in paths:
  q=Path(p);state[p]=q.exists()
  if q.exists(): t=B/p.lstrip('/');t.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(q,t)
 (B/'state.json').write_text(json.dumps(state))
 rollback=(D/'rollback-7.101.py').read_text()
 (B/'rollback.py').write_text(rollback)
 dest=Path('/opt/chrony-4.3/sbin');dest.mkdir(parents=True,exist_ok=True)
 shutil.copy2(S/'usr/sbin/chronyd',dest/'chronyd')
 shutil.copy2(S/'usr/bin/chronyc','/usr/local/bin/chronyc')
 shutil.copy2(S/'usr/share/doc/chrony/copyright',dest.parent/'COPYRIGHT')
 for name in ('RPI_Tracker.service','Quectel_RTK.service','rtc-sync.service','chrony.service'):shutil.copy2(D/name,Path('/etc/systemd/system')/name)
 shutil.copy2(D/'chrony-7.101.conf','/etc/chrony/chrony.conf')
 run(str(dest/'chronyd'),'-p','-f','/etc/chrony/chrony.conf')
 run('systemd-run','--unit=time-alignment-rollback','--on-active=5m','/usr/bin/python3',str(B/'rollback.py'),'--if-unconfirmed')
 run('systemctl','disable','--now','ntp.service')
 run('systemctl','mask','ntp.service')
 run('systemctl','daemon-reload')
 run('systemctl','enable','chrony.service')
 run('systemctl','restart','chrony.service')
 print('Chrony activated; verify and confirm within five minutes. Tracker not restarted; new unit loaded.')
finally:
 run('sync');run('mount','-o','remount,ro','/')
