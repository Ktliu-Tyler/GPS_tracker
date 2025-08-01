import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
import csv
import os
import struct
import time
from datetime import datetime

class GpsCsvLogger(Node):
    def __init__(self):
        super().__init__('gps_csv_logger')

        # 訂閱 /fix topic (GPS位置資料)
        self.subscription_fix = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # 訂閱 /vel topic (GPS速度資料)
        self.subscription_vel = self.create_subscription(
            TwistStamped,
            '/vel',
            self.velocity_callback,
            10
        )
        
        self.subscription_fix  # 避免未使用警告
        self.subscription_vel  # 避免未使用警告

        # 建立存檔資料夾及檔案
        base_dir = os.path.expanduser('./gps_logs')
        os.makedirs(base_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        self.filepath = os.path.join(base_dir, f"gps_logCSV_{timestamp}.csv")
        self.file = open(self.filepath, 'w', newline='', buffering=1)  # 行緩衝
        self.writer = csv.writer(self.file)

        # 寫入 CSV 標題列
        self.writer.writerow([
            "Time Stamp", "ID", "Extended", "Dir", "Bus", "LEN",
            "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
            "D9", "D10", "D11", "D12"
        ])

        self.get_logger().info(f"CSV logger started: {self.filepath}")

    def write_fake_can(self, can_id: int, float_val: float):
        timestamp = int(time.time() * 1e6)  # 微秒 timestamp
        packed = struct.pack('<d', float_val)  # double precision, 8 bytes little-endian
        data_bytes = [f"{b:02X}" for b in packed]
        data_bytes += ['00'] * (12 - len(data_bytes))  # 補齊到12欄

        self.writer.writerow([
            timestamp,
            f"{can_id:08X}",
            'false',
            'Rx',
            0,
            8,  # DLC = 8 bytes for double
        ] + data_bytes)

        self.file.flush()  # 立即寫入硬碟

        self.get_logger().info(
            f"Write CAN ID 0x{can_id:03X} → {float_val:.15f} → bytes: {' '.join(data_bytes[:8])}"
        )

    def gps_callback(self, msg: NavSatFix):
        self.write_fake_can(0x130, msg.latitude)
        self.write_fake_can(0x131, msg.longitude)
        self.write_fake_can(0x132, msg.altitude)
        
        # 記錄當前時間 (格式: 201507122256)
        current_time_num = float(datetime.now().strftime("%Y%m%d%H%M%S"))
        self.write_fake_can(0x140, current_time_num)

    def velocity_callback(self, msg: TwistStamped):
        # 除錯：確認有接收到速度資料
        self.get_logger().info(f"Received velocity data: x={msg.twist.linear.x:.6f}, y={msg.twist.linear.y:.6f}, z={msg.twist.linear.z:.6f}")
        
        # 記錄三個方向的線速度 (m/s)
        self.write_fake_can(0x133, msg.twist.linear.x)   # X方向速度 (東向)
        self.write_fake_can(0x134, msg.twist.linear.y)   # Y方向速度 (北向)
        self.write_fake_can(0x135, msg.twist.linear.z)   # Z方向速度 (上向)
        
        # 記錄三個方向的角速度 (rad/s)
        self.write_fake_can(0x136, msg.twist.angular.x)  # X軸角速度 (roll)
        self.write_fake_can(0x137, msg.twist.angular.y)  # Y軸角速度 (pitch)
        self.write_fake_can(0x138, msg.twist.angular.z)  # Z軸角速度 (yaw)
        
        # 計算總速度大小 (m/s)
        velocity_magnitude = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
        self.write_fake_can(0x139, velocity_magnitude)   # 總速度大小

    def destroy_node(self):
        self.get_logger().info("Closing CSV file.")
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
