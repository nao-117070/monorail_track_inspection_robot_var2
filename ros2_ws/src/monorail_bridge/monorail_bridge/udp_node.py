import rclpy
from rclpy.node import Node
import socket
import struct
import threading
from std_msgs.msg import Int16, Int16MultiArray

class UDPBridgeNode(Node):
    def __init__(self):
        super().__init__('udp_bridge_node')
        
        # ネットワーク設定
        self.teensy_ip = '192.168.1.10'
        self.teensy_port = 8888
        self.pc_ip = '192.168.1.20'
        self.pc_port = 8888
        
        # UDPソケットの初期化
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.pc_ip, self.pc_port))
        
        # ROS 2 Publisher (センサデータ用) & Subscriber (指令値用)
        # センサデータは [pot0, pot1, pot2, pot3, rpm0, rpm1, rpm2, rpm3] の8要素
        self.sensor_pub = self.create_publisher(Int16MultiArray, '/sensor_data', 10)
        self.cmd_sub = self.create_subscription(Int16, '/cmd_rpm', self.cmd_callback, 10)
        
        # UDP受信用の別スレッドを起動
        self.receive_thread = threading.Thread(target=self.receive_udp, daemon=True)
        self.receive_thread.start()
        
        self.get_logger().info("UDP Bridge Node Started. Waiting for data...")

    def cmd_callback(self, msg):
        # 1. iPadから受信したRPM指令をバイナリ (int16_t, 2バイト) にパック
        # '<h' はリトルエンディアンのC言語のshort (int16_t) を意味します
        cmd_data = struct.pack('<h', msg.data)
        
        # TeensyへUDP送信
        self.sock.sendto(cmd_data, (self.teensy_ip, self.teensy_port))
        self.get_logger().info(f"[TX] Sent Target RPM: {msg.data}")

    def receive_udp(self):
        while True:
            try:
                # 2. Teensyからのバイナリデータを受信 (16バイト)
                data, addr = self.sock.recvfrom(1024)
                
                if len(data) == 16: # int16_t(2バイト) × 8個 = 16バイト
                    # バイナリデータをアンパック (8個のint16_t)
                    # '<8h' はリトルエンディアンで8個のshortを意味します
                    unpacked_data = struct.unpack('<8h', data)
                    
                    # ROS 2 トピックとしてパブリッシュ
                    msg = Int16MultiArray()
                    msg.data = list(unpacked_data)
                    self.sensor_pub.publish(msg)
                    
            except Exception as e:
                self.get_logger().error(f"UDP Receive Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()