#!/bin/bash
set -e
cd "$(dirname "$0")"
PY="$HOME/miniconda3/bin/python3"
[ -x "$PY" ] || PY=python3
"$PY" -m PyInstaller --noconfirm --clean --windowed \
  --name ArmClient --osx-bundle-identifier com.richard.armclient \
  --add-data "web_viewer:web_viewer" \
  --hidden-import arm_sim --hidden-import web_viewer.server \
  --hidden-import paho.mqtt.client --hidden-import serial.tools.list_ports \
  client_app.py
rm -rf "$HOME/Applications/机械臂客户端.app"
ditto dist/ArmClient.app "$HOME/Applications/机械臂客户端.app"
echo "已安装：$HOME/Applications/机械臂客户端.app"
