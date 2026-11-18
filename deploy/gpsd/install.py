#!/usr/bin/python3
"""Install staged, verified gpsd; confirm manually before rollback timer expires."""
import hashlib,json,subprocess
from pathlib import Path
SRC=Path(__file__).resolve().parent
DEST=Path('/opt/gpsd-3.22-rtcm3x')
BACK=Path('/var/lib/rpi-tracker/gpsd-rtcm3x-backup')
DROP=Path('/etc/systemd/system/gpsd.service.d/rtcm3x.conf')
def run(*args): return subprocess.run(args,check=True,capture_output=True,text=True).stdout.strip()
def yes(*args): return subprocess.run(args,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
def main():
    if BACK.exists(): raise SystemExit('Existing migration backup: inspect before redeploying')
    info=json.loads((SRC/'build-info.json').read_text())
    binary=SRC/'sbin/gpsd'
    if hashlib.sha256(binary.read_bytes()).hexdigest()!=info['binary_sha256']: raise SystemExit('Binary checksum mismatch')
    print(run(str(binary),'-V'))
    ro='ro' in run('findmnt','-no','OPTIONS','/').split(',')
    holds=set(run('apt-mark','showhold').split())
    packages=[]
    for pkg in ('gpsd','gpsd-clients','libgps28','python3-gps'):
        p=subprocess.run(['dpkg-query','-W','-f=${Status}',pkg],capture_output=True,text=True)
        if p.stdout=='install ok installed': packages.append(pkg)
    link=Path('/dev/gpsd0')
    import configparser
    c=configparser.ConfigParser(interpolation=None);c.read('/etc/GPS_config.ini')
    target=c.get('RTK_CONFIG','rtk_port').strip()
    if link.exists() or link.is_symlink(): raise SystemExit('Existing /dev/gpsd0: inspect before migration')
    state={'root_was_readonly':ro,'dropin_existed':DROP.exists(),'rtk_enabled':yes('systemctl','is-enabled','Quectel_RTK.service'),'rtk_active':yes('systemctl','is-active','Quectel_RTK.service'),'new_holds':[p for p in packages if p not in holds],'gpsd_link_created':True,'gpsd_link_target':target}
    for u in ('gpsd.service','gpsd.socket'): state[u+'_enabled_before']=yes('systemctl','is-enabled',u)
    run('mount','-o','remount,rw','/')
    BACK.mkdir(parents=True,mode=0o700)
    (BACK/'state.json').write_text(json.dumps(state,indent=2))
    (BACK/'gpsd.defaults').write_bytes(Path('/etc/default/gpsd').read_bytes())
    if DROP.exists(): (BACK/'rtcm3x.conf').write_bytes(DROP.read_bytes())
    DEST.mkdir(parents=True,exist_ok=True);(DEST/'sbin').mkdir(exist_ok=True)
    for name in ('launch.py','rollback.py','build.sh','build-info.json','0001-accept-rtcm3x.patch','COPYING','README.md','sbin/gpsd'):
        (DEST/name).write_bytes((SRC/name).read_bytes())
    (DEST/'sbin/gpsd').chmod(0o755)
    run('systemd-run','--unit=gpsd-migration-rollback','--on-active=5m','/usr/bin/python3',str(DEST/'rollback.py'),'--if-unconfirmed')
    try:
        run('systemctl','disable','--now','Quectel_RTK.service')
        DROP.parent.mkdir(parents=True,exist_ok=True);DROP.write_bytes((SRC/'rtcm3x.conf').read_bytes())
        run('systemctl','daemon-reload')
        run('systemctl','enable','gpsd.service','gpsd.socket')
        run('systemctl','restart','gpsd.service')
        if state['new_holds']: run('apt-mark','hold',*state['new_holds'])
        print('Installed; validate and create confirmed marker within 5 minutes.')
    except Exception:
        subprocess.run(['/usr/bin/python3',str(DEST/'rollback.py')],check=True)
        raise
    finally:
        if ro: run('sync');run('mount','-o','remount,ro','/')
if __name__=='__main__':main()
