#!/bin/bash

CONFIG_FILE='/etc/GPS_config.ini'
# 读取配置函数
get_config() {
  local section=$1 key=$2
  grep -A 50 "^\[$section\]" "$CONFIG_FILE" | \
  grep -m1 "^$key" | \
  awk -F '=' '{print $2}' | \
  sed 's/[;#].*$//' | sed 's/^[ \t]*//;s/[ \t]*$//' | tr -d '\r'
}

if [ $(get_config "RTK_CONFIG" "enable") = "True" ]; then
    python3 /etc/RPI_APRS/Quectel_RTK_Service/Quectel_RTK.py -P $(get_config "RTK_CONFIG" "rtk_port") -u $(get_config "RTK_CONFIG" "username") -p $(get_config "RTK_CONFIG" "password") $(get_config "RTK_CONFIG" "ntrip_server") $(get_config "RTK_CONFIG" "ntrip_port") $(get_config "RTK_CONFIG" "mountpoint")
fi

