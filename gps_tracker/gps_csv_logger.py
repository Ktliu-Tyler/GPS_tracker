import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
import struct
import time
from datetime import datetime

class GpsCsvLogger(Node):
    def __init__(self):
        super().__init__('gps_csv_logger')

        # 訂閱 /fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        self.subscription  # 避免未使用警告

        # 建立存檔資料夾及檔案
        base_dir = os.path.expanduser('./gps_logs')
        os.makedirs(base_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(base_dir, f"gps_log_{timestamp}.csv")
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
        self.write_fake_can(0x100, msg.latitude)
        self.write_fake_can(0x101, msg.longitude)
        self.write_fake_can(0x102, msg.altitude)

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
