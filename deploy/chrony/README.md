# 7.101 授时与服务对齐（2026-11-18）

已完成：RPI_Tracker.service、备用 Quectel_RTK.service 和 rtc-sync.service 对齐 9.100；chrony 接收 gpsd SHM 0 的 GPS 和 SHM 1 的 PPS，GPS 设 noselect 仅为 PPS 提供整秒锁定参考，优先 PPS，备用服务器为 10.0.7.1。RTC 同步依赖改为 After=chrony.service，原 rtc-sync.timer 保留。

## 安装方式

7.101 的 apt 安装模拟存在依赖冲突，因此使用 9.100 上 dpkg -V chrony 校验无异常的 chrony 4.3 程序。运行库检查及 chronyd/chronyc -v 在目标机成功。chronyd 位于 /opt/chrony-4.3/sbin/chronyd，chronyc 位于 /usr/local/bin/chronyc，不覆盖系统共享库或修改 apt 包依赖。该独立程序不由 apt 升级维护，后续安全更新需要人工验证与替换。

本目录保存实际配置及一次性安装记录脚本 install-7.101.py。脚本需要 /tmp/align-time7 下的参考文件和程序，以及 /tmp/chrony-config 下的本目录配置；不能直接在其他主机照搬。备份：/var/lib/rpi-tracker/time-alignment-20261118/。安装有五分钟未确认自动回滚；本次完成验证后写入 confirmed 并停止该定时器。

## 最终状态及验证边界

- chrony 实际运行，开机启用；ntp 停用并 mask，避免两个进程同时调整系统时间。
- chronyc sources 显示 ^* 10.0.7.1，tracking 为 Leap status: Normal。GPS 数据已有输入但时间异常（约 34 小时偏差），已用 noselect 排除其直接选源，PPS 尚无可用样本，因此当前通过网关授时；未宣称 GPS/PPS 已经被选中。需要接收机恢复有效卫星时间后再验证 PPS 选源。
- RPI_Tracker 已重启，新环境变量生效；旧 RTK 服务保持 disabled/inactive。
- rtc-sync.service 在 chrony 同步后执行验证，结果见 validation-10.0.7.101.json。
- 9.100 的 gpsd_webdashboard、monitor_tcp_gps 已 disable --now，状态 inactive/disabled。原状态记录：/var/lib/rpi-tracker/service-cleanup-20261118/before.json。
- 整机重启验证未执行。

## 此次部署修正

旧 chrony SysV 空服务标记 active/exited，enable --now 不会实际启动新 unit，必须显式 restart。systemd RuntimeDirectory 在 root 服务启动时恢复 root 所有者，与 chronyd 降权用户冲突；改为 ExecStartPre 使用 install 创建 _chrony 所有、0750 权限的 /run/chrony，恢复命令套接字和运行数据写入。

迁移期间网络曾中断，初次只读恢复失败，初次自动回滚又被只读状态阻断。恢复连接后完成显式启动、权限修复和重新验收。回滚脚本已更换为 rollback-7.101.py，避免 systemctl enable/disable 触发旧 SysV 脚本；原始备份保留。

## 检查与回滚

```sh
systemctl show chrony ntp -p ActiveState -p SubState -p MainPID
chronyc sources -v
chronyc tracking
sudo journalctl -u chrony -n 30 --no-pager
systemctl show rtc-sync -p Result -p ExecMainStatus
findmnt -no OPTIONS /
```

手动恢复迁移前服务配置及 ntpd：

```sh
sudo python3 /var/lib/rpi-tracker/time-alignment-20261118/rollback.py
```

回滚不删除独立安装的 chrony 程序，也不影响已部署的 gpsd RTCM3X 补丁。恢复 Tracker unit 后若需立即恢复其进程环境，应再执行 systemctl restart RPI_Tracker。
