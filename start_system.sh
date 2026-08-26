#!/bin/bash

# ROS 2とワークスペースの環境設定を読み込む
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Documents/monorail_track_inspection_robot_var2/ros2_ws/install/setup.bash

echo "========================================="
echo "  Monorail System Starting..."
echo "========================================="

# 1. ROSBridgeサーバーをバックグラウンド起動
echo "-> Starting ROSBridge (WebSocket)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
PID_ROSBRIDGE=$!

# 2. UDP通信ノードをバックグラウンド起動
echo "-> Starting UDP Bridge Node..."
ros2 run monorail_bridge udp_node &
PID_UDP=$!

# 3. HTTPサーバーをバックグラウンド起動
echo "-> Starting HTTP Server (Port: 8000)..."
cd ~/Documents/monorail_track_inspection_robot_var2/web_ui
python3 -m http.server 8000 > /dev/null 2>&1 &
PID_HTTP=$!

echo "========================================="
echo "  All processes are running!"
echo "  iPad Browser URL : http://192.168.1.20:8000"
echo "  Press [Ctrl+C] to stop everything."
echo "========================================="

# Ctrl+C (SIGINT) が押されたら、バックグラウンドのプロセスを全て終了する
trap "echo -e '\nStopping all processes...'; kill $PID_ROSBRIDGE $PID_UDP $PID_HTTP; exit" SIGINT

# スクリプトが終了しないように待機
wait