# gpsd 直接接入千寻 NTRIP

## 方案与边界

gpsd 3.22 的补丁将源列表中的 `RTCM3X` 作为 RTCM3 格式别名接受，不修改差分负载。gpsd 独占 RTK_CONFIG.rtk_port（当前为 /dev/ttyUSB0），通过 /dev/gpsd0 同时读取 GNSS、写入差分，并关联 /dev/pps0。原 Quectel_RTK.service 停用；Tracker/Traccar 继续使用 gpsd 输出。FLOAT/FIX 由接收机解算决定，本补丁不保证 FIX。

安装位置 /opt/gpsd-3.22-rtcm3x，systemd 覆盖文件 /etc/systemd/system/gpsd.service.d/rtcm3x.conf。发行版 /usr/sbin/gpsd 和 /etc/default/gpsd 保留；后者的设备参数不用于自定义启动。launch.py 从设备自身 /etc/GPS_config.ini 读取账号，不复制另一台设备的配置。使用 -p 允许差分写入且避免主动配置接收机，不能换成禁止写入的 -b。

## 构建与部署

源码、补丁、二进制 SHA256 见 build-info.json；build.sh 提供源码构建入口。9.100 曾发现系统编译器校验不符，构建使用隔离的已校验编译器及开发依赖，未覆盖系统库。已有二进制只能用于经过架构与动态依赖检查的兼容 armhf 主机。

将本目录文件及构建产物 sbin/gpsd、上游 COPYING 放入远端同一个暂存目录。不要将其他主机的 validation.json 当作目标机验证结果。

```sh
# 示例：暂存目录 /tmp/gpsd-stage；首先检查运行库与版本
ldd /tmp/gpsd-stage/sbin/gpsd
/tmp/gpsd-stage/sbin/gpsd -V
sudo python3 /tmp/gpsd-stage/install.py
```

install.py 拒绝覆盖已有迁移备份；备份服务启用状态、旧覆盖文件、默认配置和新增 hold 列表。停止旧 RTK 后启动新 gpsd，启用开机启动，仅锁定实际已安装的 gpsd/gpsd-clients/libgps28/python3-gps 包，恢复原只读挂载状态。安装有五分钟自动回滚保护；验证未通过时不要确认。

## 验证与确认

至少观察 90 秒：实际运行路径和 SHA256 正确、NRestarts 不增加；USB 串口只有 gpsd 占用；持续收到 RTCM3 和新 GGA；差分龄期不过度增长；Tracker HTTP 输出更新。安装 chrony 的设备还应确认 chronyc sources -v 中 PPS 被选中。首次部署需记录断线重连验证；重启和长期稳定性测试单独记录，不由短时采样替代。

验证记录按主机保存为 validation-<IP>.json（原 validation.json 属于 9.100）。通过后：

```sh
sudo mount -o remount,rw /
sudo touch /var/lib/rpi-tracker/gpsd-rtcm3x-backup/confirmed
sudo systemctl stop gpsd-migration-rollback.timer
sync
sudo mount -o remount,ro /
```

以上挂载恢复命令适用于原本只读的 Pi-Star；原本可写的设备保持可写。

## 日常检查

```sh
systemctl show gpsd -p ActiveState -p MainPID -p NRestarts
sudo journalctl -u gpsd -n 100 --no-pager
gpsmon localhost:2947:/dev/gpsd0
gpspipe -r localhost:2947:/dev/gpsd0 | grep --line-buffered -E '^\$..GGA,'
gpspipe -w localhost:2947 | grep --line-buffered '"class":"RTCM3"'
sudo ss -tnp | grep ':8002'
sudo fuser -v /dev/ttyUSB0 /dev/ttyAMA0
chronyc sources -v
apt-mark showhold
sudo readlink -f /proc/$(systemctl show gpsd -p MainPID --value)/exe
```

gpsmon 不指定设备时会收到 NTRIP 数据并提示 No monitor matches RTCM104V3，这是显示限制。不要直接打开正在使用的串口。GGA quality=4 为 FIX、5 为 FLOAT；gpsd JSON status=3 为 FIX、4 为 FLOAT；mode=3 只是三维定位。TCP ESTAB 不等于持续收到差分。服务日志和 DEVICES JSON 可能包含带密码的 NTRIP URL，分享前脱敏。

## 回滚与升级

```sh
sudo python3 /opt/gpsd-3.22-rtcm3x/rollback.py
```

回滚恢复原服务覆盖与启用状态，重启发行版 gpsd，恢复旧 RTK 服务，移除本次新增 hold，恢复原只读状态。备份在 /var/lib/rpi-tracker/gpsd-rtcm3x-backup。原有 NTRIP 配置没有被修改。

apt hold 防止常规升级替换相关发行版包，自定义二进制另存 /opt；hold 不阻止管理员显式覆盖或 Pi-Star 整机升级。将来升级需先在测试环境验证兼容补丁、串口写入、GGA、断线恢复与 PPS，再替换并更新校验和。不要盲目解除锁定。

## 2026-11-18 部署记录

- 10.0.9.100：原 validation.json 记录；已验证 FLOAT、差分龄期约 1 秒、断线恢复及 chrony PPS。
- 10.0.7.101：见 validation-10.0.7.101.json。沿用本机 qxymtq002，95 秒收到 303 条 RTCM3、96 条 GGA，自动重启 0。另用 15 秒系统调用采样确认 USB 写入 4094 字节、NTRIP 上报 GGA 3 次。Tracker 输出更新、Traccar 队列 0。仅 gpsd 和 libgps28 实际安装并被 hold；未安装的 gpsd-clients/python3-gps 未安装或锁定。
- 7.101 验证限制：迁移前旧服务日志已有 GGA quality=6；迁移后卫星数为 0、quality=6、接收机时间错误，因此本次保留已验证服务链路的部署，但不宣称差分被用于有效卫星解算或 RTK FIX。未安装 chrony，未验证 PPS 授时；未在该机注入断线或整机重启。恢复有效卫星信号后须补做差分龄期与 FLOAT/FIX 验证。
