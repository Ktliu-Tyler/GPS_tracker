import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray, String
import can
import struct
import time
from datetime import datetime

class CanPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # 儲存最新的速度資料，用於GPS callback中發送
        self.latest_velocity = None
        
        # 訂閱GPS位置資料
        self.subscription_fix = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # 訂閱GPS速度資料
        self.subscription_vel = self.create_subscription(
            TwistStamped,
            '/vel',
            self.velocity_callback,
            10
        )
        
        # 訂閱原始GPS字串資料
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
        
        self.get_logger().info("CAN Publisher Node Started - GPS/Velocity + Timestamp + NMEA String")
        self.get_logger().info("Publishing: NavSatFix + TwistStamped + NMEA String → CAN + Timestamp (0x100)")
    
    def send_timestamp(self):
        """每秒發送時間戳到CAN匯流排"""
        try:
            # 計算時間戳
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            # 準備資料 (little endian, 參考timer_can_pub.py)
            timestamp_data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little')
            
            # 發送CAN訊息
            can_msg = can.Message(
                arbitration_id=0x100, 
                data=timestamp_data, 
                is_extended_id=False
            )
            self.bus.send(can_msg)
            
            # 發布ROS2訊息
            ros_msg = UInt8MultiArray()
            ros_msg.data = list(timestamp_data)
            self.timestamp_publisher.publish(ros_msg)
            
            # Log資訊
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.get_logger().info(
                f'Timestamp sent (0x100) - Time: {current_time}, '
                f'MS since midnight: {ms_since_midnight}, Days since 1984: {days_since_1984}'
            )
            
        except can.CanError as e:
            self.get_logger().error(f'CAN Error sending timestamp: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending timestamp: {e}')

    def gps_callback(self, msg: NavSatFix):
        try:
            # Ecumaster 格式編碼 - 0x400 (GPS基本資訊)
            # 只使用 NavSatFix 中可用的資料：latitude, longitude
            lat_scaled = int(msg.latitude * 10**7)
            lat_bytes = struct.pack('<i', lat_scaled)
            
            lon_scaled = int(msg.longitude * 10**7)
            lon_bytes = struct.pack('<i', lon_scaled)
            
            # 組合成 8-byte 資料 (緯度4字節 + 經度4字節)
            gps_basic_data = lat_bytes + lon_bytes
            
            # 高度：使用 NavSatFix 的 altitude
            height_scaled = int(msg.altitude) if not float('inf') == msg.altitude and not float('-inf') == msg.altitude else 0
            height_bytes = struct.pack('<h', height_scaled)
            

            # 建立CAN訊息
            messages = [
                can.Message(arbitration_id=0x400, data=gps_basic_data, is_extended_id=False),      # GPS基本資訊
                can.Message(arbitration_id=0x401, data=height_bytes, is_extended_id=False),   # GPS擴展資訊
            ]

            # 將 9 個 float64 (72 bytes) 以 9 個 CAN frame 傳送，每 frame 8 bytes
            cov_bytes = b''.join([struct.pack('<d', v) for v in msg.position_covariance])
            for i in range(9):
                frame_data = cov_bytes[i*8:(i+1)*8]  # 每個 covariance float64 拆成一個 frame
                messages.append(
                    can.Message(arbitration_id=0x410 + i, data=frame_data, is_extended_id=False)
                )
            
            frame_data = struct.pack('<B', msg.position_covariance_type) + b'\x00'*7
            messages.append(
                can.Message(arbitration_id=0x410 + 9, data=frame_data, is_extended_id=False)
            )

            for m in messages:
                self.bus.send(m)

            self.get_logger().info(
                f"Sent GPS CAN frames: "
                f"Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, "
                f"Alt={msg.altitude:.1f}m"
                f", Covariance Type={msg.position_covariance_type}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send GPS CAN messages: {e}")
            
    def velocity_callback(self, msg: TwistStamped):
        try:
            # 儲存最新的速度資料，供GPS callback使用
            self.latest_velocity = msg
            # 使用 0x404-0x40A 作為擴展速度資料
            vx_scaled = int(msg.twist.linear.x * 1000)  # 以mm/s為單位
            vy_scaled = int(msg.twist.linear.y * 1000)
            vz_scaled = int(msg.twist.linear.z * 1000)
            vx_bytes = struct.pack('<i', vx_scaled) + b'\x00\x00\x00\x00'  # 4字節速度+4字節填充
            vy_bytes = struct.pack('<i', vy_scaled) + b'\x00\x00\x00\x00'
            vz_bytes = struct.pack('<i', vz_scaled) + b'\x00\x00\x00\x00'
            
            # 角速度詳細資料 (以 0.001 rad/s 為單位)
            wx_scaled = int(msg.twist.angular.x * 1000)
            wy_scaled = int(msg.twist.angular.y * 1000) 
            wz_scaled = int(msg.twist.angular.z * 1000)
            wx_bytes = struct.pack('<i', wx_scaled) + b'\x00\x00\x00\x00'
            wy_bytes = struct.pack('<i', wy_scaled) + b'\x00\x00\x00\x00'
            wz_bytes = struct.pack('<i', wz_scaled) + b'\x00\x00\x00\x00'
            
            # 總速度大小
            velocity_magnitude = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
            vmag_scaled = int(velocity_magnitude * 1000)  # mm/s
            vmag_bytes = struct.pack('<i', vmag_scaled) + b'\x00\x00\x00\x00'

            # 建立CAN訊息 (Ecumaster格式 + 擴展速度資料)
            messages = [
                # 擴展速度資料 (詳細各方向資料)
                can.Message(arbitration_id=0x402, data=vx_bytes, is_extended_id=False),        # X方向線性速度
                can.Message(arbitration_id=0x403, data=vy_bytes, is_extended_id=False),        # Y方向線性速度
                can.Message(arbitration_id=0x404, data=vz_bytes, is_extended_id=False),        # Z方向線性速度
                can.Message(arbitration_id=0x405, data=wx_bytes, is_extended_id=False),        # X軸角速度
                can.Message(arbitration_id=0x406, data=wy_bytes, is_extended_id=False),        # Y軸角速度
                can.Message(arbitration_id=0x407, data=wz_bytes, is_extended_id=False),        # Z軸角速度
                can.Message(arbitration_id=0x408, data=vmag_bytes, is_extended_id=False),      # 總速度大小
            ]

            for m in messages:
                self.bus.send(m)

            # 計算總速度用於log
            speed_kmh = velocity_magnitude * 3.6

            self.get_logger().info(
                f"Sent Velocity CAN frames (Ecumaster + Extended): "
                f"Linear=({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}) m/s, "
                f"Angular=({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f}) rad/s, "
                f"Total Speed={speed_kmh:.2f}km/h"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send velocity CAN messages: {e}")

    def gps_string_callback(self, msg: String):
        """處理原始 NMEA GPS 字串並發送到 CAN"""
        try:
            nmea_string = msg.data.strip()
            
            # 將 NMEA 字串轉換為 bytes
            nmea_bytes = nmea_string.encode('ascii')
            
            # 計算需要多少個 CAN frame (每個 frame 最多 8 bytes)
            # 使用 0x500-0x5FF 範圍作為 NMEA 字串傳輸
            frame_count = (len(nmea_bytes) + 7) // 8  # 向上取整
            
            messages = []
            
            # 第一個 frame 包含總長度和 frame 數量資訊
            header_data = struct.pack('<HH', len(nmea_bytes), frame_count) + b'\x00' * 4
            messages.append(
                can.Message(arbitration_id=0x500, data=header_data, is_extended_id=False)
            )
            
            # 分割字串為多個 8-byte frames
            for i in range(frame_count):
                start_idx = i * 8
                end_idx = min(start_idx + 8, len(nmea_bytes))
                frame_data = nmea_bytes[start_idx:end_idx]
                
                # 如果不足 8 bytes，用 0x00 填充
                if len(frame_data) < 8:
                    frame_data += b'\x00' * (8 - len(frame_data))
                
                messages.append(
                    can.Message(arbitration_id=0x501 + i, data=frame_data, is_extended_id=False)
                )
            
            # 發送所有 CAN 訊息
            for m in messages:
                self.bus.send(m)
            
            # 解析 NMEA 字串類型用於日誌
            sentence_type = "Unknown"
            if nmea_string.startswith('$'):
                parts = nmea_string.split(',')
                if len(parts) > 0:
                    sentence_type = parts[0][3:]  # 去掉 "$GN" 前綴
            
            self.get_logger().info(
                f"Sent NMEA string to CAN: Type={sentence_type}, "
                f"Length={len(nmea_bytes)} bytes, Frames={frame_count+1}, "
                f"Data='{nmea_string[:50]}{'...' if len(nmea_string) > 50 else ''}'"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to send NMEA string to CAN: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

