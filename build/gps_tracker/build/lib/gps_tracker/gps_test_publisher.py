#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import math
import time

class GPSTestPublisher(Node):
    def __init__(self):
        super().__init__('gps_test_publisher')
        
        # 建立GPS資料發布者
        self.publisher = self.create_publisher(NavSatFix, '/fix', 10)
        
        # 建立定時器，每秒發布一次
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        
        # 測試路徑點（台北市區幾個地點）
        self.test_points = [
            {'lat': 25.01365, 'lon': 121.52861, 'alt': 32.5, 'name': '台北101'},
            {'lat': 25.04776, 'lon': 121.51713, 'alt': 35.2, 'name': '台北車站'},
            {'lat': 25.04200, 'lon': 121.50664, 'alt': 28.8, 'name': '西門町'},
            {'lat': 25.05888, 'lon': 121.52578, 'alt': 45.1, 'name': '松山機場'},
            {'lat': 25.02800, 'lon': 121.54340, 'alt': 38.9, 'name': '信義區'},
        ]
        
        self.current_point_index = 0
        self.get_logger().info('GPS測試發布器已啟動，開始發送測試資料到 /fix topic')
    
    def publish_gps_data(self):
        """發布GPS測試資料"""
        msg = NavSatFix()
        
        # 設定header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        # 設定GPS狀態
        msg.status.status = 0  # GPS fix available
        msg.status.service = 1  # GPS service
        
        # 取得當前測試點
        current_point = self.test_points[self.current_point_index]
        
        # 設定座標
        msg.latitude = current_point['lat']
        msg.longitude = current_point['lon']
        msg.altitude = current_point['alt']
        
        # 設定協方差（模擬GPS精度）
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        msg.position_covariance_type = 1  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # 發布訊息
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f'發送GPS測試資料: {current_point["name"]} - '
            f'緯度: {msg.latitude:.6f}, 經度: {msg.longitude:.6f}, 高度: {msg.altitude:.1f}m'
        )
        
        # 切換到下一個測試點
        self.current_point_index = (self.current_point_index + 1) % len(self.test_points)

class GPSTestCircularPath(Node):
    """建立圓形路徑的GPS測試發布器"""
    def __init__(self):
        super().__init__('gps_test_circular_path')
        
        self.publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer = self.create_timer(0.5, self.publish_gps_data)  # 每0.5秒發布一次
        
        # 調整後的圓形路徑參數
        self.center_lat = 25.0134755  # 中心緯度
        self.center_lon = 121.528274  # 中心經度
        self.radius = 0.00028         # 半徑，確保不會超出邊界
        self.angle = 0.0
        self.angle_step = math.pi / 18  # 每次轉10度
        
        self.get_logger().info('GPS圓形路徑測試發布器已啟動')
    
    def publish_gps_data(self):
        """發布圓形路徑GPS資料"""
        msg = NavSatFix()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        msg.status.status = 0
        msg.status.service = 1
        
        # 計算圓形路徑上的座標
        msg.latitude = self.center_lat + self.radius * math.cos(self.angle)
        msg.longitude = self.center_lon + self.radius * math.sin(self.angle)
        msg.altitude = 32.5 + 5.0 * math.sin(self.angle * 2)  # 高度也有變化
        
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = 1
        
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f'圓形路徑GPS: 緯度: {msg.latitude:.6f}, 經度: {msg.longitude:.6f}, '
            f'角度: {math.degrees(self.angle):.1f}°'
        )
        
        # 更新角度
        self.angle += self.angle_step
        if self.angle >= 2 * math.pi:
            self.angle = 0.0


import re

class GPSTestFilePublisher(Node):
    def __init__(self, file_path):
        super().__init__('gps_test_file_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.file_path = file_path
        with open(self.file_path, 'r') as f:
            self.lines = f.readlines()
        self.index = 0
        self.timer = self.create_timer(0.05, self.publish_gps_data)
        self.get_logger().info(f'GPS檔案測試發布器已啟動，資料來源: {self.file_path}')

    def publish_gps_data(self):
        while self.index < len(self.lines):
            line = self.lines[self.index]
            self.index += 1
            if line.startswith('$GNGGA') or line.startswith('$GNRMC'):
                lat, lon = self.parse_nmea_line(line)
                if lat is not None and lon is not None:
                    msg = NavSatFix()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps'
                    msg.status.status = 0
                    msg.status.service = 1
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = 0.0
                    msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    msg.position_covariance_type = 1
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        f'檔案GPS: 緯度: {lat:.6f}, 經度: {lon:.6f}'
                    )
                    break  # 一次只發一筆
        if self.index >= len(self.lines):
            self.get_logger().info('GPS檔案資料已全部發送完畢')
            self.timer.cancel()

    def parse_nmea_line(self, line):
        lat, lon = None, None
        if line.startswith('$GNGGA'):
            parts = line.strip().split(',')
            if len(parts) > 5 and parts[2] and parts[4]:
                lat = self.nmea_to_decimal(parts[2], parts[3])
                lon = self.nmea_to_decimal(parts[4], parts[5])
        elif line.startswith('$GNRMC'):
            parts = line.strip().split(',')
            if len(parts) > 5 and parts[3] and parts[5]:
                lat = self.nmea_to_decimal(parts[3], parts[4])
                lon = self.nmea_to_decimal(parts[5], parts[6])
        return lat, lon

    def nmea_to_decimal(self, value, direction):
        if not value or value == 'nan':
            return None
        d, m = re.match(r"(\d+)(\d\d\.\d+)", value).groups()
        decimal = float(d) + float(m) / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'circle':
        node = GPSTestCircularPath()
    elif len(sys.argv) > 2 and sys.argv[1] == 'file':
        node = GPSTestFilePublisher(sys.argv[2])
    else:
        node = GPSTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()