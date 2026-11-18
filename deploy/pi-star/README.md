# Pi-Star 主页内嵌

`tracker.conf` 将 `/tracker/` 代理到 `127.0.0.1:5050`。主页标题下显示可折叠的 iframe，管理页不添加面板。模板使用相对 URL，兼容原来的 5050 直连入口。关闭代理响应缓冲，避免 Pi-Star 只读根文件系统导致大 JS 文件下载中断。

部署到已安装 RPI Tracker 的 Pi-Star：

```sh
scp -r deploy/pi-star pi-star@10.0.9.100:/tmp/rpi-tracker-embed
ssh pi-star@10.0.9.100 'sudo python3 /tmp/rpi-tracker-embed/install.py'
```

脚本只修改远端主页、两个 HTML 模板和独立 Nginx 配置；每次部署先备份到 `/home/pi-star/tracker-embed-backups/`，验证 PHP/Nginx 语法后重载 Nginx，失败则恢复。若根文件系统原为只读，结束时恢复只读。无需重启 GPS/APRS 进程。可重复运行；Pi-Star 更新覆盖主页后也可重新运行。

2026-11-18 首次部署前备份：`/home/pi-star/tracker-embed-backups/20261118-104706-753954`。需要完全回退时，先将根文件系统临时重新挂载为可写，将该目录中的 `var/www/dashboard/index.php` 和 `etc/RPI_Tracker/web_templates/` 下两个模板还原至相同绝对路径，删除 `/etc/nginx/default.d/tracker.conf`，执行 `nginx -t` 和 `systemctl reload nginx`，最后 `sync` 并恢复只读挂载。

验证：主页只有一个 iframe；`/tracker/`、两份图表 JS、四个数据接口及配置页返回 200；代理 JS 与 5050 原始内容一致；浏览器中图表及实时状态正常显示；5050 直连仍正常。未提交配置保存表单，避免触发服务重启。
