#!/bin/bash


mount -o remount,rw / ; sudo mount -o remount,rw /boot

apt-get update
apt-get -y install i2c-tools python3-smbus python3-pip python3-pil libjpeg-dev zlib1g-dev libfreetype6-dev liblcms2-dev libopenjp2-7 libtiff5 gpsd
python3 -m pip install --upgrade setuptools adafruit-circuitpython-ssd1306 adafruit-python-shell luma.oled pillow gps3 aprs psutil fastapi uvicorn
#安装必要依赖


git reset --hard
git pull origin main

echo "Code pulled on $(date)"

# 检查 /etc/GPS_config.ini 是否存在
if [ -f "/etc/GPS_config.ini" ]; then
    echo "/etc/GPS_config.ini already exists. Skipping copy."
else
    echo "/etc/GPS_config.ini does not exist. Copying..."
    cp ./GPS_config.ini /etc/GPS_config.ini
    echo "Copied ./GPS_config.ini to /etc/GPS_config.ini."
fi
mkdir /etc/RPI_Tracker
cp -r * /etc/RPI_Tracker

# 定义服务文件路径
SERVICE_FILE="/etc/systemd/system/RPI_Tracker.service"
SCRIPT_PATH="/etc/RPI_Tracker/RPI_Tracker.sh"
LOG_DIR="/var/log"

# 确保日志目录存在
mkdir -p "$LOG_DIR"

# 检查并创建脚本目录
if [ ! -d "/etc/RPI_Tracker" ]; then
    mkdir -p "/etc/RPI_Tracker"
    echo "Created directory /etc/RPI_Tracker"
fi

# 检查脚本是否存在
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: $SCRIPT_PATH does not exist. Please ensure your script is in place."
    exit 1
fi

# 写入服务文件内容
echo "Creating systemd service file..."
cat <<EOF | sudo tee "$SERVICE_FILE"
[Unit]
Description=GPS Tracker Reporter Service
After=network.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=/etc/RPI_Tracker
ExecStart=/usr/bin/python3 RPI_Tracker.py
ExecStop=/bin/bash -c "pkill -f RPI_Tracker.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 设置正确的权限
sudo chmod 644 "$SERVICE_FILE"
echo "Service file created at $SERVICE_FILE"

# 重新加载 systemd 服务
sudo systemctl daemon-reload

# 启用服务开机自启
sudo systemctl enable RPI_Tracker.service
echo "Service enabled to start at boot."

# 启动服务
sudo systemctl start RPI_Tracker.service
echo "Service started."

# 检查服务状态
sudo systemctl status RPI_Tracker.service --no-pager



sync ; sudo sync ; sudo sync ; sudo mount -o remount,ro / ; sudo mount -o remount,ro /boot


