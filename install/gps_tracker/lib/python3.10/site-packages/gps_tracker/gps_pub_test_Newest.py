#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header, String
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
    """建立圓形路徑的GPS測試發布器，同時發送GPS位置、速度和NMEA字串"""
    def __init__(self):
        super().__init__('gps_test_circular_path')
        
        # 建立發布者 (參考 gps_can_pub.py 的訂閱 topics)
        self.gps_publisher = self.create_publisher(NavSatFix, '/fix', 10)          # GPS 位置
        self.velocity_publisher = self.create_publisher(TwistStamped, '/vel', 10)  # 速度資料
        self.gps_str_publisher = self.create_publisher(String, '/gps_str', 10)     # NMEA 字串
        
        self.timer = self.create_timer(0.5, self.publish_data)  # 每0.5秒發布一次
        
        # 調整後的圓形路徑參數
        self.center_lat = 25.0134755  # 中心緯度
        self.center_lon = 121.528274  # 中心經度
        self.radius = 0.00028         # 半徑，確保不會超出邊界
        self.angle = 0.0
        self.angle_step = math.pi / 18  # 每次轉10度
        
        # 速度相關參數
        self.previous_lat = self.center_lat
        self.previous_lon = self.center_lon
        self.previous_time = time.time()
        
        self.get_logger().info('GPS圓形路徑測試發布器已啟動 - 發送GPS位置 + 速度 + NMEA字串')
        self.get_logger().info('📡 發布 Topics: /fix (GPS位置) + /vel (速度) + /gps_str (NMEA字串)')
    
    def publish_data(self):
        """同時發布GPS位置、速度和NMEA字串資料"""
        current_time = time.time()
        
        # 計算圓形路徑上的座標
        current_lat = self.center_lat + self.radius * math.cos(self.angle)
        current_lon = self.center_lon + self.radius * math.sin(self.angle)
        current_alt = 32.5 + 5.0 * math.sin(self.angle * 2)  # 高度也有變化
        
        # 1. 發布GPS位置資料 (/fix)
        self.publish_gps_data(current_lat, current_lon, current_alt)
        
        # 2. 發布速度資料 (/vel)
        self.publish_velocity_data(current_lat, current_lon, current_alt, current_time)
        
        # 3. 發布NMEA字串 (/gps_str)
        self.publish_nmea_string(current_lat, current_lon, current_alt)
        
        # 更新狀態
        self.previous_lat = current_lat
        self.previous_lon = current_lon
        self.previous_time = current_time
        
        # 更新角度
        self.angle += self.angle_step
        if self.angle >= 2 * math.pi:
            self.angle = 0.0
    
    def publish_gps_data(self, lat, lon, alt):
        """發布GPS位置資料到 /fix"""
        msg = NavSatFix()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        msg.status.status = 0  # GPS fix available
        msg.status.service = 1  # GPS service
        
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = 1  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(msg)
    
    def publish_velocity_data(self, lat, lon, alt, current_time):
        """發布速度資料到 /vel"""
        msg = TwistStamped()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 計算時間差
        dt = current_time - self.previous_time
        if dt > 0:
            # 計算地理座標變化 (簡化的線性近似)
            dlat = lat - self.previous_lat
            dlon = lon - self.previous_lon
            
            # 轉換為米/秒 (粗略估算: 1度緯度 ≈ 111000m, 1度經度 ≈ 111000m * cos(lat))
            lat_to_m = 111000.0
            lon_to_m = 111000.0 * math.cos(math.radians(lat))
            
            # 線性速度 (m/s)
            vx = (dlon * lon_to_m) / dt  # 東西方向
            vy = (dlat * lat_to_m) / dt  # 南北方向
            vz = 0.0  # 垂直方向 (簡化為0)
            
            # 模擬圓形運動的理論速度 (作為參考)
            angular_velocity = self.angle_step / 0.5  # rad/s
            tangential_speed = angular_velocity * self.radius * lat_to_m  # 切線速度
            
            # 使用理論速度，但加入一些變化
            msg.twist.linear.x = tangential_speed * math.cos(self.angle + math.pi/2)
            msg.twist.linear.y = tangential_speed * math.sin(self.angle + math.pi/2)
            msg.twist.linear.z = 0.1 * math.sin(self.angle * 3)  # 微小的垂直速度變化
        else:
            # 初始化時的速度
            msg.twist.linear.x = 2.0  # m/s
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
        
        # 角速度 (rad/s) - 模擬車輛轉彎
        msg.twist.angular.x = 0.05 * math.sin(self.angle * 2)  # 微小的側傾
        msg.twist.angular.y = 0.03 * math.cos(self.angle * 1.5)  # 微小的俯仰
        msg.twist.angular.z = self.angle_step / 0.5  # 轉彎角速度
        
        self.velocity_publisher.publish(msg)
    
    def publish_nmea_string(self, lat, lon, alt):
        """發布NMEA字串到 /gps_str"""
        # 生成 GPGGA 句子 (Global Positioning System Fix Data)
        utc_time = datetime.now().strftime("%H%M%S.%f")[:-4]  # HHMMSS.SS
        
        # 轉換座標格式 (度分格式)
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_dir = 'N' if lat >= 0 else 'S'
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        
        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_dir = 'E' if lon >= 0 else 'W'
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        
        # 生成 GPGGA 句子
        gga_fields = [
            "GPGGA",
            utc_time,               # UTC 時間
            lat_str,                # 緯度
            lat_dir,                # 緯度方向
            lon_str,                # 經度
            lon_dir,                # 經度方向
            "1",                    # GPS 品質指示 (1=GPS fix)
            "08",                   # 使用的衛星數量
            "1.0",                  # 水平精度稀釋
            f"{alt:.1f}",           # 天線高度
            "M",                    # 高度單位
            "0.0",                  # 大地水準面高度
            "M",                    # 大地水準面高度單位
            "",                     # DGPS 資料年齡
            ""                      # DGPS 基站 ID
        ]
        
        # 組合句子 (不含校驗和)
        sentence_without_checksum = "$" + ",".join(gga_fields)
        
        # 計算校驗和
        checksum = 0
        for char in sentence_without_checksum[1:]:  # 跳過 '$'
            checksum ^= ord(char)
        
        # 完整的 NMEA 句子
        nmea_sentence = f"{sentence_without_checksum}*{checksum:02X}\r\n"
        
        # 發布NMEA字串
        nmea_msg = String()
        nmea_msg.data = nmea_sentence
        self.gps_str_publisher.publish(nmea_msg)
        
        self.get_logger().info(
            f'圓形路徑: GPS({lat:.6f},{lon:.6f}) 高度({alt:.1f}m) '
            f'角度({math.degrees(self.angle):.1f}°) NMEA: {nmea_sentence.strip()}'
        )

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
        self.get_logger().info('🚗 CAN訊號: 0x100(時間戳) + 0x400-0x419(GPS) + 0x402-0x408(速度) + 0x500-0x5FF(NMEA) + 0x190-0x714(蓄電池+逆變器)')
    
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
            
            # GPS 擴展資訊 (0x401) - 僅高度
            height_scaled = int(current_alt)
            height_bytes = struct.pack('<h', height_scaled)
            gps_extended_data = height_bytes + b'\x00\x00\x00\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x401, data=gps_extended_data, is_extended_id=False))
            
            # GPS Position Covariance (0x410-0x418)
            position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            for i, cov_value in enumerate(position_covariance):
                cov_data = struct.pack('<d', cov_value)  # float64 (8 bytes)
                messages.append(can.Message(arbitration_id=0x410 + i, data=cov_data, is_extended_id=False))
            
            # GPS Position Covariance Type (0x419)
            cov_type = 1  # Approximated
            cov_type_data = struct.pack('<B', cov_type) + b'\x00\x00\x00\x00\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x419, data=cov_type_data, is_extended_id=False))
            
            # 3. 速度資料 (0x402-0x408)
            speed = 10.0 + 5.0 * math.sin(self.time_counter * 0.5)  # 5-15 m/s
            vel_x = speed * math.cos(self.angle * 2)
            vel_y = speed * math.sin(self.angle * 2) * 0.3
            vel_z = 0.1 * math.sin(self.time_counter)
            
            # 各軸線性速度 (0x402-0x404) - mm/s
            vel_x_scaled = int(vel_x * 1000)
            vel_y_scaled = int(vel_y * 1000)
            vel_z_scaled = int(vel_z * 1000)
            messages.append(can.Message(arbitration_id=0x402, data=struct.pack('<i', vel_x_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x403, data=struct.pack('<i', vel_y_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x404, data=struct.pack('<i', vel_z_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            
            # 各軸角速度 (0x405-0x407) - mrad/s
            ang_x = 0.2 * math.sin(self.time_counter * 1.5)
            ang_y = 0.15 * math.cos(self.time_counter * 1.2)
            ang_z = 0.5 * math.sin(self.angle * 3)
            ang_x_scaled = int(ang_x * 1000)
            ang_y_scaled = int(ang_y * 1000)
            ang_z_scaled = int(ang_z * 1000)
            messages.append(can.Message(arbitration_id=0x405, data=struct.pack('<i', ang_x_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x406, data=struct.pack('<i', ang_y_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            messages.append(can.Message(arbitration_id=0x407, data=struct.pack('<i', ang_z_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))
            
            # 總速度大小 (0x408) - mm/s
            total_speed = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
            total_speed_scaled = int(total_speed * 1000)
            messages.append(can.Message(arbitration_id=0x408, data=struct.pack('<i', total_speed_scaled) + b'\x00\x00\x00\x00', is_extended_id=False))

            # 4. 蓄電池假資料
            # Cell Voltage (0x190)
            cell_voltages = [min(255, max(0, int((3.6 + 0.2 * math.sin(self.time_counter + i)) / 0.02))) for i in range(7)]
            cell_data = struct.pack('<B', 0) + struct.pack('<7B', *cell_voltages)
            messages.append(can.Message(arbitration_id=0x190, data=cell_data, is_extended_id=False))
            
            # Accumulator Status (0x290) - status(1) + temp(2) + voltage(4)
            status = 1  # OK
            temp_scaled = int((25.0 + 10.0 * math.sin(self.time_counter * 0.3)) / 0.125)  # 0.125°C/LSB
            temp_scaled = min(32767, max(-32768, temp_scaled))
            voltage_v = 48.0 + 4.0 * math.sin(self.time_counter * 0.2)  # 44-52V
            voltage_scaled = int(voltage_v * 1024)  # 1/1024V/LSB
            voltage_scaled = min(4294967295, max(0, voltage_scaled))  # unsigned 32-bit
            status_data = struct.pack('<BhI', status, temp_scaled, voltage_scaled) + b'\x00'
            messages.append(can.Message(arbitration_id=0x290, data=status_data, is_extended_id=False))
            
            # Accumulator Temperature (0x390)
            temps = [min(255, max(0, int((30.0 + 5.0 * math.sin(self.time_counter + i * 0.5)) / 0.5))) for i in range(7)]
            temp_data = struct.pack('<B', 0) + struct.pack('<7B', *temps)
            messages.append(can.Message(arbitration_id=0x390, data=temp_data, is_extended_id=False))
            
            # Accumulator State (0x490) - soc(1) + current(2) + capacity(4)
            self.battery_soc = max(10, min(100, self.battery_soc - 0.01))  # 緩慢放電
            current_a = 20.0 + 10.0 * math.sin(self.time_counter * 0.4)  # 10-30A
            current_scaled = int(current_a / 0.01)  # 10mA/LSB
            current_scaled = min(32767, max(-32768, current_scaled))
            capacity_ah = 50.0  # 50Ah
            capacity_scaled = int(capacity_ah / 0.01)  # 10mAh/LSB (signed)
            capacity_scaled = min(2147483647, max(-2147483648, capacity_scaled))  # signed 32-bit
            state_data = struct.pack('<Bhi', int(self.battery_soc), current_scaled, capacity_scaled)
            messages.append(can.Message(arbitration_id=0x490, data=state_data, is_extended_id=False))
            
            # Accumulator Heartbeat (0x710)
            messages.append(can.Message(arbitration_id=0x710, data=b'\x7F', is_extended_id=False))
            
            # 5. NMEA 字串測試資料 (0x500-0x5FF)
            nmea_sentence = f"$GPGGA,123456.00,{abs(current_lat):.6f},{'N' if current_lat >= 0 else 'S'},{abs(current_lon):.6f},{'E' if current_lon >= 0 else 'W'},1,08,1.0,{current_alt:.1f},M,0.0,M,,*65\r\n"
            nmea_bytes = nmea_sentence.encode('ascii')
            nmea_length = len(nmea_bytes)
            frame_count = (nmea_length + 7) // 8  # 每個frame 8 bytes
            
            # NMEA Header (0x500)
            header_data = struct.pack('<HH', nmea_length, frame_count) + b'\x00\x00\x00\x00'
            messages.append(can.Message(arbitration_id=0x500, data=header_data, is_extended_id=False))
            
            # NMEA 字串內容 (0x501-0x5FF)
            for frame_idx in range(frame_count):
                start_idx = frame_idx * 8
                end_idx = min(start_idx + 8, nmea_length)
                frame_data = nmea_bytes[start_idx:end_idx]
                if len(frame_data) < 8:
                    frame_data += b'\x00' * (8 - len(frame_data))  # 填充到8字節
                messages.append(can.Message(arbitration_id=0x501 + frame_idx, data=frame_data, is_extended_id=False))
            
            # 6. 逆變器假資料
            for i in range(1, 5):  # FL, FR, RL, RR (1-4)
                # Inverter Status (0x191-0x194)
                status_word = 0x1234 + i  # 不同逆變器有不同狀態
                torque_nm = 50.0 + 30.0 * math.sin(self.time_counter + i)  # 20-80 Nm
                torque_scaled = int(torque_nm * 1000)  # 1/1000 rated torque/LSB
                torque_scaled = min(32767, max(-32768, torque_scaled))
                speed_rpm = int(1500 + 500 * math.sin(self.time_counter * 0.6 + i))  # 1000-2000 RPM
                speed_rpm = min(32767, max(-32768, speed_rpm))
                inv_status_data = struct.pack('<Hhh', status_word, torque_scaled, speed_rpm) + b'\x00\x00'
                messages.append(can.Message(arbitration_id=0x190+i, data=inv_status_data, is_extended_id=False))
                
                # Inverter State (0x291-0x294)
                dc_voltage_v = 300.0 + 20.0 * math.sin(self.time_counter + i * 0.3)  # 280-320V
                dc_current_a = 50.0 + 20.0 * math.sin(self.time_counter + i * 0.7)  # 30-70A
                dc_voltage_scaled = int(dc_voltage_v * 100)  # 1/100V/LSB
                dc_current_scaled = int(dc_current_a * 100)  # 1/100A/LSB
                dc_voltage_scaled = min(65535, max(0, dc_voltage_scaled))
                dc_current_scaled = min(65535, max(0, dc_current_scaled))
                inv_state_data = struct.pack('<HH', dc_voltage_scaled, dc_current_scaled) + b'\x00\x00\x00\x00'
                messages.append(can.Message(arbitration_id=0x290+i, data=inv_state_data, is_extended_id=False))
                
                # Inverter Temperature (0x391-0x394)
                mos_temp_c = 60.0 + 15.0 * math.sin(self.time_counter + i)  # 45-75°C
                mcu_temp_c = 45.0 + 10.0 * math.sin(self.time_counter + i * 0.5)  # 35-55°C
                motor_temp_c = 70.0 + 20.0 * math.sin(self.time_counter + i * 0.8)  # 50-90°C
                mos_temp_scaled = int(mos_temp_c * 10)  # 0.1°C/LSB
                mcu_temp_scaled = int(mcu_temp_c * 10)  # 0.1°C/LSB
                motor_temp_scaled = int(motor_temp_c * 10)  # 0.1°C/LSB
                mos_temp_scaled = min(32767, max(-32768, mos_temp_scaled))
                mcu_temp_scaled = min(32767, max(-32768, mcu_temp_scaled))
                motor_temp_scaled = min(32767, max(-32768, motor_temp_scaled))
                inv_temp_data = struct.pack('<hhh', mos_temp_scaled, mcu_temp_scaled, motor_temp_scaled) + b'\x00\x00'
                messages.append(can.Message(arbitration_id=0x390+i, data=inv_temp_data, is_extended_id=False))
                
                # Inverter Heartbeat (0x711-0x714)
                messages.append(can.Message(arbitration_id=0x710+i, data=b'\x7F', is_extended_id=False))
            
            # 發送所有CAN訊息
            for msg in messages:
                self.bus.send(msg)
            
            speed_kmh = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2) * 3.6  # 總速度 km/h
            self.get_logger().info(
                f'發送 {len(messages)} 個CAN訊息 - GPS座標({current_lat:.6f},{current_lon:.6f}) '
                f'高度({current_alt:.1f}m) 速度({speed_kmh:.1f}km/h) 電量({self.battery_soc:.0f}%) '
                f'NMEA長度({nmea_length}字節,{frame_count}框)'
            )
            
        except Exception as e:
            self.get_logger().error(f"發送CAN資料失敗: {e}")

class CAC_csv_pub(Node):
    def __init__(self, file_path):
        super().__init__('can_csv_publisher')
        self.file_path = file_path
        
        # 建立CAN匯流排連接
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.can_available = True
            self.get_logger().info("CAN匯流排連接成功")
        except Exception as e:
            self.can_available = False
            self.get_logger().error(f"CAN匯流排連接失敗: {e}")
            return
        
        # 讀取CSV檔案
        try:
            self.can_messages = self.parse_csv_file()
            if not self.can_messages:
                self.get_logger().error("沒有有效的CAN訊息可發送")
                return
        except Exception as e:
            self.get_logger().error(f"讀取CSV檔案失敗: {e}")
            return
        
        # 設定發送參數
        self.message_index = 0
        self.loop_count = 0
        
        # 建立定時器，每100ms發送一個訊息
        self.timer = self.create_timer(0.1, self.send_next_message)
        
        self.get_logger().info(f'CAN CSV發布器已啟動 - 檔案: {self.file_path}')
        self.get_logger().info(f'總共載入 {len(self.can_messages)} 個CAN訊息')
    
    def parse_csv_file(self):
        """解析CSV檔案並提取CAN訊息"""
        import csv
        can_messages = []
        
        try:
            with open(self.file_path, 'r', encoding='utf-8') as csvfile:
                # 嘗試自動偵測CSV格式
                sample = csvfile.read(1024)
                csvfile.seek(0)
                sniffer = csv.Sniffer()
                delimiter = sniffer.sniff(sample).delimiter
                
                # 讀取CSV檔案
                reader = csv.DictReader(csvfile, delimiter=delimiter)
                
                for row_num, row in enumerate(reader, start=2):  # 從第2行開始計數（第1行是標題）
                    try:
                        # 支援多種欄位名稱格式
                        can_id = None
                        data = None
                        delay = 0.1  # 預設延遲100ms
                        
                        # 嘗試不同的欄位名稱
                        for id_field in ['ID', 'id', 'CAN_ID', 'can_id', 'Address', 'address']:
                            if id_field in row and row[id_field]:
                                can_id = row[id_field].strip()
                                break
                        
                        for data_field in ['Data', 'data', 'DATA', 'Payload', 'payload', 'Bytes', 'bytes']:
                            if data_field in row and row[data_field]:
                                data = row[data_field].strip()
                                break
                        
                        for delay_field in ['Delay', 'delay', 'Time', 'time', 'Interval', 'interval']:
                            if delay_field in row and row[delay_field]:
                                try:
                                    delay = float(row[delay_field])
                                except ValueError:
                                    pass
                                break
                        
                        if can_id is None or data is None:
                            self.get_logger().warn(f"第 {row_num} 行：缺少必要欄位 (ID 或 Data)")
                            continue
                        
                        # 解析CAN ID
                        if can_id.startswith('0x') or can_id.startswith('0X'):
                            parsed_id = int(can_id, 16)
                        else:
                            parsed_id = int(can_id)
                        
                        # 解析資料
                        parsed_data = self.parse_can_data(data)
                        if parsed_data is None:
                            self.get_logger().warn(f"第 {row_num} 行：無效的資料格式 '{data}'")
                            continue
                        
                        # 創建CAN訊息
                        can_msg = {
                            'id': parsed_id,
                            'data': parsed_data,
                            'delay': delay,
                            'row': row_num
                        }
                        can_messages.append(can_msg)
                        
                    except Exception as e:
                        self.get_logger().warn(f"第 {row_num} 行處理失敗: {e}")
                        continue
                
        except FileNotFoundError:
            self.get_logger().error(f"找不到檔案: {self.file_path}")
            return []
        except Exception as e:
            self.get_logger().error(f"解析CSV檔案時發生錯誤: {e}")
            return []
        
        self.get_logger().info(f"成功解析 {len(can_messages)} 個CAN訊息")
        return can_messages
    
    def parse_can_data(self, data_str):
        """解析CAN資料字串"""
        try:
            # 移除所有空格、冒號、破折號
            clean_data = data_str.replace(' ', '').replace(':', '').replace('-', '').replace(',', '').strip()
            
            # 確保字串長度為偶數
            if len(clean_data) % 2 != 0:
                self.get_logger().warn(f"資料長度不是偶數，將補0: '{data_str}'")
                clean_data = '0' + clean_data
            
            # 轉換為位元組
            data_bytes = bytes.fromhex(clean_data)
            
            # CAN資料長度限制為8位元組
            if len(data_bytes) > 8:
                self.get_logger().warn(f"CAN資料長度超過8位元組，將截取前8位元組: '{data_str}'")
                data_bytes = data_bytes[:8]
            
            return data_bytes
            
        except ValueError as e:
            self.get_logger().error(f"資料格式錯誤: '{data_str}', 錯誤: {e}")
            return None
    
    def send_next_message(self):
        """發送下一個CAN訊息"""
        if not self.can_available or not self.can_messages:
            return
        
        # 檢查是否已發送完所有訊息
        if self.message_index >= len(self.can_messages):
            self.loop_count += 1
            self.message_index = 0
            self.get_logger().info(f"完成第 {self.loop_count} 次循環，重新開始發送")
        
        # 取得當前訊息
        msg_data = self.can_messages[self.message_index]
        
        try:
            # 建立CAN訊息
            message = can.Message(
                arbitration_id=msg_data['id'],
                data=msg_data['data'],
                is_extended_id=(msg_data['id'] > 0x7FF)  # 判斷是否為擴展幀
            )
            
            # 發送訊息
            self.bus.send(message)
            
            # 格式化資料顯示
            data_hex = ' '.join([f'{b:02X}' for b in msg_data['data']])
            self.get_logger().info(
                f"[{self.message_index+1}/{len(self.can_messages)}] "
                f"發送 ID=0x{msg_data['id']:X}, Data=[{data_hex}], DLC={len(msg_data['data'])}"
            )
            
            # 更新延遲時間（如果指定）
            if msg_data['delay'] != 0.1:  # 如果不是預設延遲
                self.timer.cancel()
                self.timer = self.create_timer(msg_data['delay'], self.send_next_message)
            
        except Exception as e:
            self.get_logger().error(f"發送CAN訊息失敗: {e}")
        
        # 移到下一個訊息
        self.message_index += 1
    
    def destroy_node(self):
        """關閉節點時清理資源"""
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
            self.get_logger().info("CAN匯流排連接已關閉")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'circle':
        node = GPSTestCircularPath()
    elif len(sys.argv) > 2 and sys.argv[1] == 'file':
        node = GPSTestFilePublisher(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1] == 'all':
        node = AllSystemTestPublisher()
    elif len(sys.argv) > 2 and sys.argv[1] == 'csv':
        node = CAC_csv_pub(sys.argv[2])
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
