#!/usr/bin/env python3
"""Run with sudo on Pi-Star; backs up changes and restores read-only root."""
import datetime
import os
from pathlib import Path
import shutil
import subprocess


def run(*args):
    subprocess.run(args, check=True)


def main():
    if os.geteuid() != 0:
        raise SystemExit('Run with sudo python3 install.py')
    source = Path(__file__).resolve().parent
    homepage = Path('/var/www/dashboard/index.php')
    templates = Path('/etc/RPI_Tracker/web_templates')
    nginx = Path('/etc/nginx/default.d/tracker.conf')
    panel = (source / 'tracker-panel.html').read_text()
    html = homepage.read_text()
    begin = '<!-- RPI_TRACKER_PANEL_BEGIN -->'
    end = '<!-- RPI_TRACKER_PANEL_END -->'
    if begin in html:
        first = html.index(begin)
        last = html.index(end, first) + len(end)
        html = html[:first] + panel.rstrip() + html[last:]
    else:
        anchor = '<?php\n// Output some default features'
        if html.count(anchor) != 1:
            raise SystemExit('Unexpected Pi-Star homepage structure; no changes made')
        html = html.replace(anchor, panel + '\n' + anchor, 1)

    changes = {homepage: html, nginx: (source / 'tracker.conf').read_text()}
    index = templates / 'index.html'
    changes[index] = (index.read_text().replace('src="/static/', 'src="static/')
                      .replace('href="/config"', 'href="config"')
                      .replace("fetch('/", "fetch('"))
    config = templates / 'config_edit.html'
    changes[config] = (config.read_text().replace('action="/config"', 'action="config"')
                       .replace('href="/"', 'href="./"'))
    was_ro = any(line.split()[1] == '/' and 'ro' in line.split()[3].split(',')
                 for line in Path('/proc/mounts').read_text().splitlines())
    backup = Path('/home/pi-star/tracker-embed-backups') / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    originals = {}
    try:
        if was_ro:
            run('mount', '-o', 'remount,rw', '/')
        backup.mkdir(parents=True, mode=0o700)
        for path in changes:
            saved = backup / str(path).lstrip('/')
            if path.exists():
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(str(path), str(saved))
                originals[path] = saved
            else:
                originals[path] = None
        try:
            for path, content in changes.items():
                path.write_text(content)
            run('php', '-l', str(homepage))
            run('nginx', '-t')
            run('systemctl', 'reload', 'nginx')
        except Exception:
            for path, saved in originals.items():
                if saved is None:
                    if path.exists():
                        path.unlink()
                else:
                    shutil.copy2(str(saved), str(path))
            run('nginx', '-t')
            run('systemctl', 'reload', 'nginx')
            raise
        print('Installed. Backup: {}'.format(backup))
    finally:
        if was_ro:
            run('sync')
            run('mount', '-o', 'remount,ro', '/')


if __name__ == '__main__':
    main()
