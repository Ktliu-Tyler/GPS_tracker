#!/usr/bin/env python3
"""
優化版 CAN Publisher
- 減少 CAN frames 數量
- 加入頻率限制
- 不發送 covariance
- 加入重試機制
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray, String
import can
import struct
import time
from datetime import datetime

class CanPublisherOptimized(Node):
    def __init__(self):
        super().__init__('gps_can_pub_node')
        
        # CAN Bus 初始化
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # CAN 發送配置
        self.can_send_delay = 0.005  # 每個 frame 之間延遲 5ms
        self.can_retry_count = 5     # 失敗時重試次數
        self.can_retry_delay = 0.02  # 重試之間延遲 20ms
        
        # 發送頻率限制 (throttle)
        self.last_gps_send_time = 0.0
        self.last_vel_send_time = 0.0
        self.gps_send_interval = 0.2  # GPS 最快 200ms 發送一次 (5Hz)
        self.vel_send_interval = 0.2  # Velocity 最快 200ms 發送一次 (5Hz)
        
        # 訂閱 GPS 位置資料
        self.subscription_fix = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # 訂閱 GPS 速度資料
        self.subscription_vel = self.create_subscription(
            TwistStamped,
            '/vel',
            self.velocity_callback,
            10
        )
        
        # 訂閱原始 GPS 字串資料
        self.subscription_gps_str = self.create_subscription(
            String,
            '/gps_str',
            self.gps_string_callback,
            10
        )
        
        # 建立時間戳發布者 (ROS2 topic)
        self.timestamp_publisher = self.create_publisher(
            UInt8MultiArray, 
            'can_timestamp', 
            10
        )
        
        # 建立時間戳定時器 (每秒發送一次)
        self.timestamp_timer = self.create_timer(1.0, self.send_timestamp)
        
        self.get_logger().info("CAN Publisher Optimized Node Started")
        self.get_logger().info("Configuration:")
        self.get_logger().info(f"  GPS rate: {1/self.gps_send_interval:.1f} Hz")
        self.get_logger().info(f"  Velocity rate: {1/self.vel_send_interval:.1f} Hz")
        self.get_logger().info(f"  CAN delay: {self.can_send_delay*1000:.1f} ms")
        self.get_logger().info(f"  Retry count: {self.can_retry_count}")
    
    def send_can_message_safe(self, msg, retry_count=None):
        """安全地發送 CAN 訊息，包含重試機制"""
        if retry_count is None:
            retry_count = self.can_retry_count
        
        for attempt in range(retry_count):
            try:
                self.bus.send(msg, timeout=0.1)
                return True
            except can.CanError as e:
                if "Transmit buffer full" in str(e) and attempt < retry_count - 1:
                    time.sleep(self.can_retry_delay)
                    continue
                else:
                    if attempt == retry_count - 1:
                        self.get_logger().warning(f'CAN send failed after {retry_count} attempts: {e}')
                    return False
            except Exception as e:
                self.get_logger().error(f'Unexpected error sending CAN: {e}')
                return False
        return False
    
    def send_timestamp(self):
        """每秒發送時間戳到 CAN 匯流排"""
        try:
            # 計算時間戳
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            # 準備資料 (little endian)
            timestamp_data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little')
            
            # 發送 CAN 訊息
            can_msg = can.Message(
                arbitration_id=0x100, 
                data=timestamp_data, 
                is_extended_id=False
            )
            if not self.send_can_message_safe(can_msg):
                return
            
            # 發布 ROS2 訊息
            ros_msg = UInt8MultiArray()
            ros_msg.data = list(timestamp_data)
            self.timestamp_publisher.publish(ros_msg)
            
            # Log 資訊
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.get_logger().info(
                f'Timestamp sent (0x100) - Time: {current_time}, '
                f'MS: {ms_since_midnight}, Days: {days_since_1984}'
            )
            
        except can.CanError as e:
            self.get_logger().error(f'CAN Error sending timestamp: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending timestamp: {e}')

    def gps_callback(self, msg: NavSatFix):
        """處理 GPS 位置資料 - 只發送基本資訊"""
        # 頻率限制：避免發送過快
        current_time = time.time()
        if current_time - self.last_gps_send_time < self.gps_send_interval:
            return
        
        try:
            # 編碼緯度和經度
            lat_scaled = int(msg.latitude * 10**7)
            lon_scaled = int(msg.longitude * 10**7)
            
            # Frame 1: 緯度 + 經度 (0x400)
            gps_basic_data = struct.pack('<ii', lat_scaled, lon_scaled)
            
            # Frame 2: 高度 + 狀態 (0x401)
            height_scaled = int(msg.altitude) if not float('inf') == msg.altitude and not float('-inf') == msg.altitude else 0
            status = msg.status.status  # NavSatStatus
            altitude_status_data = struct.pack('<hH', height_scaled, status) + b'\x00'*4
            
            # 建立 CAN 訊息（只有 2 個 frames）
            messages = [
                can.Message(arbitration_id=0x400, data=gps_basic_data, is_extended_id=False),
                can.Message(arbitration_id=0x401, data=altitude_status_data, is_extended_id=False),
            ]

            # 發送訊息，每個之間加入延遲
            success_count = 0
            for m in messages:
                if self.send_can_message_safe(m):
                    success_count += 1
                    time.sleep(self.can_send_delay)
                else:
                    break

            if success_count == len(messages):
                self.last_gps_send_time = current_time
                self.get_logger().info(
                    f"GPS: {success_count}/{len(messages)} frames | "
                    f"Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.1f}m"
                )
            else:
                self.get_logger().warning(f"GPS: Only sent {success_count}/{len(messages)} frames")
                
        except Exception as e:
            self.get_logger().error(f"GPS callback error: {e}")
            
    def velocity_callback(self, msg: TwistStamped):
        """處理速度資料 - 只發送線性速度"""
        # 頻率限制
        current_time = time.time()
        if current_time - self.last_vel_send_time < self.vel_send_interval:
            return
        
        try:
            # 計算總速度
            velocity_magnitude = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
            
            # 速度資料 (mm/s)
            vx_scaled = int(msg.twist.linear.x * 1000)
            vy_scaled = int(msg.twist.linear.y * 1000)
            vz_scaled = int(msg.twist.linear.z * 1000)
            vmag_scaled = int(velocity_magnitude * 1000)
            
            # Frame 1: VX + VY (0x403)
            vel_xy_data = struct.pack('<ii', vx_scaled, vy_scaled)
            
            # Frame 2: VZ + 總速度 (0x404)
            vel_z_total_data = struct.pack('<ii', vz_scaled, vmag_scaled)
            
            # 建立 CAN 訊息（只有 2 個 frames）
            messages = [
                can.Message(arbitration_id=0x403, data=vel_xy_data, is_extended_id=False),
                can.Message(arbitration_id=0x404, data=vel_z_total_data, is_extended_id=False),
            ]

            # 發送訊息
            success_count = 0
            for m in messages:
                if self.send_can_message_safe(m):
                    success_count += 1
                    time.sleep(self.can_send_delay)
                else:
                    break

            if success_count == len(messages):
                self.last_vel_send_time = current_time
                speed_kmh = velocity_magnitude * 3.6
                self.get_logger().info(
                    f"Velocity: {success_count}/{len(messages)} frames | Speed={speed_kmh:.2f}km/h"
                )
            else:
                self.get_logger().warning(f"Velocity: Only sent {success_count}/{len(messages)} frames")
                
        except Exception as e:
            self.get_logger().error(f"Velocity callback error: {e}")

    def gps_string_callback(self, msg: String):
        """處理原始 NMEA GPS 字串並發送到 CAN"""
        try:
            nmea_string = msg.data.strip()
            
            # 只處理重要的 NMEA 句子
            if not any(x in nmea_string for x in ['GGA', 'RMC', 'VTG']):
                return
            
            # 將 NMEA 字串轉換為 bytes
            nmea_bytes = nmea_string.encode('ascii')
            
            # 計算需要多少個 CAN frame
            frame_count = (len(nmea_bytes) + 7) // 8
            
            # 限制最大 frames 數量
            if frame_count > 10:
                self.get_logger().warning(f"NMEA string too long ({len(nmea_bytes)} bytes), skipping")
                return
            
            messages = []
            
            # 第一個 frame 包含長度資訊 (0x500)
            header_data = struct.pack('<HH', len(nmea_bytes), frame_count) + b'\x00' * 4
            messages.append(
                can.Message(arbitration_id=0x500, data=header_data, is_extended_id=False)
            )
            
            # 分割字串為多個 8-byte frames
            for i in range(frame_count):
                start_idx = i * 8
                end_idx = min(start_idx + 8, len(nmea_bytes))
                frame_data = nmea_bytes[start_idx:end_idx]
                
                # 填充到 8 bytes
                if len(frame_data) < 8:
                    frame_data += b'\x00' * (8 - len(frame_data))
                
                messages.append(
                    can.Message(arbitration_id=0x501 + i, data=frame_data, is_extended_id=False)
                )
            
            # 發送所有 CAN 訊息
            success_count = 0
            for m in messages:
                if self.send_can_message_safe(m):
                    success_count += 1
                    time.sleep(self.can_send_delay)
                else:
                    break
            
            if success_count == len(messages):
                # 解析句子類型
                sentence_type = nmea_string.split(',')[0][3:] if ',' in nmea_string else "Unknown"
                self.get_logger().info(
                    f"NMEA: {success_count}/{len(messages)} frames | Type={sentence_type}, Len={len(nmea_bytes)}B"
                )
            else:
                self.get_logger().warning(f"NMEA: Only sent {success_count}/{len(messages)} frames")
            
        except Exception as e:
            self.get_logger().error(f"NMEA callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanPublisherOptimized()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
