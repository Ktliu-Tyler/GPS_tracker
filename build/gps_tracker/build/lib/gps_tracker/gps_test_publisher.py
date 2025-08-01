#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import can
import struct
import math
import time
from datetime import datetime
import re

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

class AllSystemTestPublisher(Node):
    """發送所有系統假資料的測試發布器"""
    def __init__(self):
        super().__init__('all_system_test_publisher')
        
        # 建立ROS2發布者
        self.gps_publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/vel', 10)
        
        # 建立CAN匯流排連接
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.can_available = True
            self.get_logger().info("CAN匯流排連接成功")
        except Exception as e:
            self.can_available = False
            self.get_logger().warn(f"CAN匯流排連接失敗: {e}，僅發送ROS2資料")
        
        # 建立定時器
        self.timer_gps = self.create_timer(0.1, self.publish_gps_data)      # 10Hz GPS
        self.timer_vel = self.create_timer(0.02, self.publish_velocity_data) # 50Hz 速度
        self.timer_can = self.create_timer(0.1, self.publish_can_data)      # 10Hz CAN資料
        
        # 模擬資料狀態
        self.time_counter = 0.0
        self.angle = 0.0
        self.battery_soc = 85  # 初始電量85%
        
        # 台北101附近的測試路徑
        self.center_lat = 25.01365
        self.center_lon = 121.52861
        self.radius = 0.0002
        
        self.get_logger().info('全系統測試發布器已啟動 - 發送GPS、速度、蓄電池、逆變器假資料')
        self.get_logger().info('📡 ROS2 Topics: /fix (GPS) + /vel (速度)')
        self.get_logger().info('🚗 CAN訊號: 0x100(時間戳) + 0x400-0x40A(GPS+速度) + 0x190-0x714(蓄電池+逆變器)')
    
    def publish_gps_data(self):
        """發布GPS測試資料"""
        msg = NavSatFix()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        msg.status.status = 0  # GPS fix available
        msg.status.service = 1  # GPS service
        
        # 模擬圓形移動路徑
        msg.latitude = self.center_lat + self.radius * math.cos(self.angle)
        msg.longitude = self.center_lon + self.radius * math.sin(self.angle)
        msg.altitude = 32.5 + 5.0 * math.sin(self.angle * 0.5)
        
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = 1
        
        self.gps_publisher.publish(msg)
        
        self.get_logger().debug(
            f'發送GPS: 緯度: {msg.latitude:.6f}, 經度: {msg.longitude:.6f}, 高度: {msg.altitude:.1f}m'
        )
    
    def publish_velocity_data(self):
        """發布速度測試資料"""
        msg = TwistStamped()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 模擬車辛運動
        speed = 10.0 + 5.0 * math.sin(self.time_counter * 0.5)  # 5-15 m/s
        msg.twist.linear.x = speed * math.cos(self.angle * 2)
        msg.twist.linear.y = speed * math.sin(self.angle * 2) * 0.3
        msg.twist.linear.z = 0.1 * math.sin(self.time_counter)
        
        # 模擬角速度
        msg.twist.angular.x = 0.2 * math.sin(self.time_counter * 1.5)
        msg.twist.angular.y = 0.15 * math.cos(self.time_counter * 1.2)
        msg.twist.angular.z = 0.5 * math.sin(self.angle * 3)
        
        self.velocity_publisher.publish(msg)
        
        self.get_logger().debug(
            f'發送速度: 線性=({msg.twist.linear.x:.2f}, {msg.twist.linear.y:.2f}, {msg.twist.linear.z:.2f}), '
            f'角速度=({msg.twist.angular.x:.2f}, {msg.twist.angular.y:.2f}, {msg.twist.angular.z:.2f})'
        )
        
        # 更新計數器
        self.time_counter += 0.02
        self.angle += 0.01
        if self.angle >= 2 * math.pi:
            self.angle = 0.0
    
    def publish_can_data(self):
        """發布CAN測試資料"""
        if not self.can_available:
            return
        
        try:
            messages = []
            
            # 1. 時間戳 (0x100)
            current_time = datetime.now()
            ms_since_midnight = int((current_time.hour * 3600 + current_time.minute * 60 + current_time.second) * 1000 + current_time.microsecond / 1000)
            days_since_1984 = int((current_time - datetime(1984, 1, 1)).days)
            timestamp_data = struct.pack('<IH', ms_since_midnight, days_since_1984) + b'\x00\x00'
            messages.append(can.Message(arbitration_id=0x100, data=timestamp_data, is_extended_id=False))
            
            # 2. GPS Ecumaster 格式資料
            # 計算當前GPS座標
            current_lat = self.center_lat + self.radius * math.cos(self.angle)
            current_lon = self.center_lon + self.radius * math.sin(self.angle)
            current_alt = 32.5 + 5.0 * math.sin(self.angle * 0.5)
            
            # GPS 基本資訊 (0x400) - 緯度和經度
            lat_scaled = int(current_lat * 10**7)
            lat_bytes = struct.pack('<i', lat_scaled)
            lon_scaled = int(current_lon * 10**7)
            lon_bytes = struct.pack('<i', lon_scaled)
            gps_basic_data = lat_bytes + lon_bytes
            messages.append(can.Message(arbitration_id=0x400, data=gps_basic_data, is_extended_id=False))
            
            # GPS 擴展資訊 (0x401) - 速度、高度、衛星數
            speed = 10.0 + 5.0 * math.sin(self.time_counter * 0.5)  # 5-15 m/s
            speed_kmh = speed * 3.6
            speed_scaled = int(speed_kmh / 0.036)
            speed_bytes = struct.pack('<h', speed_scaled)
            height_scaled = int(current_alt)
            height_bytes = struct.pack('<h', height_scaled)
            satellites_num = 8
            satellites_bytes = struct.pack('<B', satellites_num)
            gps_frame_idx = 0
            empty_frame_idx = 0 
            gps_status = 1
            frame_status_byte = (gps_frame_idx & 0x0F) | ((empty_frame_idx & 0x0F) << 4)
            status_byte = gps_status & 0x07
            gps_extended_data = speed_bytes + height_bytes + satellites_bytes + struct.pack('<B', frame_status_byte) + struct.pack('<B', status_byte) + b'\x00'
            messages.append(can.Message(arbitration_id=0x401, data=gps_extended_data, is_extended_id=False))
            
            # GPS 航向和角速度 (0x402)
            heading = math.degrees(self.angle) % 360
            heading_scaled = int(heading / 0.0054931640625)
            heading_bytes = struct.pack('<H', heading_scaled)
            angular_vel = 0.5 * math.sin(self.angle * 3)
            angular_vel_scaled = int(angular_vel / 0.0054931640625) if angular_vel >= 0 else int((angular_vel + 65536 * 0.0054931640625) / 0.0054931640625)
            angular_vel_bytes = struct.pack('<H', angular_vel_scaled)
            gps_heading_data = heading_bytes + angular_vel_bytes + b'\x00\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x402, data=gps_heading_data, is_extended_id=False))
            
            # GPS 加速度 (0x403)
            accel_x = 2.0 * math.sin(self.time_counter * 0.8)
            accel_y = 1.5 * math.cos(self.time_counter * 0.6)
            accel_x_scaled = int(accel_x / 0.001220703125)
            accel_y_scaled = int(accel_y / 0.001220703125)
            accel_x_bytes = struct.pack('<h', accel_x_scaled)
            accel_y_bytes = struct.pack('<h', accel_y_scaled)
            gps_accel_data = accel_x_bytes + accel_y_bytes + b'\x00\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x403, data=gps_accel_data, is_extended_id=False))
            
            # 3. 擴展速度資料 (0x404-0x40A)
            vel_x = speed * math.cos(self.angle * 2)
            vel_y = speed * math.sin(self.angle * 2) * 0.3
            vel_z = 0.1 * math.sin(self.time_counter)
            
            # 各軸線性速度 (0x404-0x406)
            vel_x_scaled = int(vel_x * 1000)
            vel_y_scaled = int(vel_y * 1000)
            vel_z_scaled = int(vel_z * 1000)
            messages.append(can.Message(arbitration_id=0x404, data=struct.pack('<i', vel_x_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x405, data=struct.pack('<i', vel_y_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x406, data=struct.pack('<i', vel_z_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            
            # 各軸角速度 (0x407-0x409)
            ang_x = 0.2 * math.sin(self.time_counter * 1.5)
            ang_y = 0.15 * math.cos(self.time_counter * 1.2)
            ang_z = angular_vel
            ang_x_scaled = int(ang_x * 1000)
            ang_y_scaled = int(ang_y * 1000)
            ang_z_scaled = int(ang_z * 1000)
            messages.append(can.Message(arbitration_id=0x407, data=struct.pack('<i', ang_x_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x408, data=struct.pack('<i', ang_y_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x409, data=struct.pack('<i', ang_z_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            
            # 速度狀態標記 (0x40A)
            vel_valid = 1  # 速度資料有效
            messages.append(can.Message(arbitration_id=0x40A, data=struct.pack('<B', vel_valid) + b'\x00\x00\x00\x00\x00\x00\x00', is_extended_id=False))

            # 4. 蓄電池假資料
            # Cell Voltage (0x190)
            cell_voltages = [min(255, max(0, int((3.6 + 0.2 * math.sin(self.time_counter + i)) / 0.02))) for i in range(7)]
            cell_data = struct.pack('<B', 0) + struct.pack('<7B', *cell_voltages)
            messages.append(can.Message(arbitration_id=0x190, data=cell_data, is_extended_id=False))
            
            # Accumulator Status (0x290)
            status = 1  # OK
            temp = min(32767, max(-32768, int((25.0 + 10.0 * math.sin(self.time_counter * 0.3)) / 0.125)))
            voltage = min(65535, max(0, int((48.0 + 4.0 * math.sin(self.time_counter * 0.2)) * 100)))  # 改為合理範圍
            status_data = struct.pack('<BhH', status, temp, voltage) + b'\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x290, data=status_data, is_extended_id=False))
            
            # Accumulator Temperature (0x390)
            temps = [min(255, max(0, int((30.0 + 5.0 * math.sin(self.time_counter + i * 0.5)) / 0.5))) for i in range(7)]
            temp_data = struct.pack('<B', 0) + struct.pack('<7B', *temps)
            messages.append(can.Message(arbitration_id=0x390, data=temp_data, is_extended_id=False))
            
            # Accumulator State (0x490)
            self.battery_soc = max(10, min(100, self.battery_soc - 0.01))  # 緩慢放電
            current = min(32767, max(-32768, int((20.0 + 10.0 * math.sin(self.time_counter * 0.4)) * 10)))  # 改為合理範圍
            capacity = min(32767, max(0, int(50.0 * 10)))  # 改為合理範圍
            state_data = struct.pack('<Bhh', int(self.battery_soc), current, capacity) + b'\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x490, data=state_data, is_extended_id=False))
            
            # Accumulator Heartbeat (0x710)
            messages.append(can.Message(arbitration_id=0x710, data=b'\x7F', is_extended_id=False))
            
            # 5. 逆變器假資料
            for i in range(1, 5):  # FL, FR, RL, RR (1-4)
                # Inverter Status (0x191-0x194)
                status_word = 0x1234
                torque = min(32767, max(-32768, int((50.0 + 30.0 * math.sin(self.time_counter + i)) * 10)))  # 改為合理範圍
                speed_rpm = min(32767, max(-32768, int(1500 + 500 * math.sin(self.time_counter * 0.6 + i))))
                inv_status_data = struct.pack('<Hhh', status_word, torque, speed_rpm) + b'\x00\x00'
                messages.append(can.Message(arbitration_id=0x190+i, data=inv_status_data, is_extended_id=False))
                
                # Inverter State (0x291-0x294)
                dc_voltage = min(65535, max(0, int((300.0 + 20.0 * math.sin(self.time_counter + i * 0.3)) * 10)))  # 改為合理範圍
                dc_current = min(65535, max(0, int((50.0 + 20.0 * math.sin(self.time_counter + i * 0.7)) * 10)))  # 改為合理範圍
                inv_state_data = struct.pack('<HH', dc_voltage, dc_current) + b'\x00\x00\x00\x00'
                messages.append(can.Message(arbitration_id=0x290+i, data=inv_state_data, is_extended_id=False))
                
                # Inverter Temperature (0x391-0x394)
                mos_temp = min(32767, max(-32768, int((60.0 + 15.0 * math.sin(self.time_counter + i)) * 10)))
                mcu_temp = min(32767, max(-32768, int((45.0 + 10.0 * math.sin(self.time_counter + i * 0.5)) * 10)))
                motor_temp = min(32767, max(-32768, int((70.0 + 20.0 * math.sin(self.time_counter + i * 0.8)) * 10)))
                inv_temp_data = struct.pack('<hhh', mos_temp, mcu_temp, motor_temp) + b'\x00\x00'
                messages.append(can.Message(arbitration_id=0x390+i, data=inv_temp_data, is_extended_id=False))
                
                # Inverter Heartbeat (0x711-0x714)
                messages.append(can.Message(arbitration_id=0x710+i, data=b'\x7F', is_extended_id=False))
            
            # 發送所有CAN訊息
            for msg in messages:
                self.bus.send(msg)
            
            self.get_logger().info(
                f'發送 {len(messages)} 個CAN訊息 - GPS座標({current_lat:.6f},{current_lon:.6f}) '
                f'速度({speed_kmh:.1f}km/h) 電量({self.battery_soc:.0f}%)'
            )
            
        except Exception as e:
            self.get_logger().error(f"發送CAN資料失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'circle':
        node = GPSTestCircularPath()
    elif len(sys.argv) > 2 and sys.argv[1] == 'file':
        node = GPSTestFilePublisher(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1] == 'all':
        node = AllSystemTestPublisher()
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