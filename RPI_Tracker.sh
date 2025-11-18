#!/bin/bash

CONFIG_FILE='/etc/GPS_config.ini'
# 读取配置函数
function get_config() {
    local section=$1
    local key=$2
    grep -A 10 "^\[$section\]" "$CONFIG_FILE" | grep "^$key" | awk -F '=' '{print $2}' | sed 's/^[ \t]*//;s/[ \t]*$//'
}
PROJECT_DIR=$(get_config "PROJECT_PATH" "PROJECT_DIR")

i2cset -y 1 0x57 0x06 0x18
#如果存在pi sugar，启用看门狗


cd "$PROJECT_DIR" || exit

echo $(get_config "APRS_Config" "SSID")
echo "start python $(date)" >> /var/log/GPS_NMEA.log
python3 RPI_Tracker.py 

