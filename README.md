# Monorail Track Inspection Robot (Var2)

モノレールの軌道桁上を走行する四輪駆動ロボットのための制御・監視システムです。
アクセスポイントを介した完全なローカルネットワーク（オフライン環境）で動作し、iPadからロボットの操縦とセンサデータのリアルタイムモニタリングを行います。

## システム構成

1. **iPad (Web UI)**
   - ブラウザ（Safari等）から操作画面にアクセス。
   - 前進・後退・停止の指令を送信し、モータの回転速度やポテンショメータの角度をリアルタイム表示。
   - `rosbridge_server` を経由してWebSocket通信を実行。
2. **Mini PC (Ubuntu / ROS 2)**
   - システムのハブとして機能（IP: `192.168.1.20`）。
   - iPadからのトピック通信（コマンド）をUDPバイナリデータに変換してTeensyへ送信。
   - TeensyからのUDPバイナリデータ（センサ情報）を受信し、ROS 2トピックとしてパブリッシュ。
3. **Teensy 4.1 (Robot Controller)**
   - ロボットに搭載されたメインマイコン（IP: `192.168.1.10` / Port: `8888`）。
   - UDPバイナリ通信で指令を受信しモータを制御。
   - 0.1秒周期で4つのモータ回転数と4つのポテンショメータ角度をUDPバイナリ通信でPCへ送信。

## ディレクトリ構成

```text
monorail_track_inspection_robot_var2/
├── teensy_firmware/      # Teensy 4.1用プログラム (PlatformIO)
├── ros2_ws/              # ミニPC用 ROS 2ワークスペース (Pythonノード)
├── web_ui/               # iPad用 Webインターフェース (HTML/JS)
├── start_system.sh       # ミニPC用 一括起動シェルスクリプト
└── README.md             # 本ドキュメント


通信仕様 (UDP バイナリ)
通信のオーバーヘッドを無くすため、文字列ではなくC言語の struct (リトルエンディアン) 形式で送受信を行います。

PC -> Teensy (指令): int16_t × 1個 (2バイト) = 目標RPM

Teensy -> PC (センサ): int16_t × 8個 (16バイト) = [Pot1~4, RPM1~4]

環境構築とセットアップ
ミニPC (Ubuntu) 側の準備

1. ROS 2 ワークスペースのビルド
cd ros2_ws
colcon build
source install/setup.bash

2. オフライン動作のための roslib.min.js の配置
（インターネット接続時に実施済み）


起動方法
ミニPCを起動し、ターミナルで以下のスクリプトを実行するだけで、必要なプロセス（ROSBridge, UDPノード, HTTPサーバー）がすべて起動します。
cd ~/Documents/monorail_track_inspection_robot_var2
./start_system.sh

iPadからのアクセス:
1. ミニPCと同じWi-Fiアクセスポイントに接続。
2. ブラウザで http://192.168.1.20:8000 にアクセス。

終了方法:
スクリプトを実行しているターミナルで Ctrl + C を押すと、全プロセスが安全に終了します。