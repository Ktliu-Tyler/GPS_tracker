import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray
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

        self.subscription_nmea = self.create_subscription(
            Sentence,
            '/nmea_sentence',
            self.nmea_callback,
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
        
        self.get_logger().info("CAN Publisher Node Started - GPS/Velocity + Timestamp")
        self.get_logger().info("Publishing: NavSatFix + TwistStamped → CAN + Timestamp (0x100)")
    
    def send_timestamp(self):
        """每秒發送時間戳到CAN匯流排"""
        try:
            # 計算時間戳
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            # 準備資料 (little endian, 參考timer_can_pub.py)
            timestamp_data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little') + b'\x00\x00'
            
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

            # Ecumaster 格式編碼 - 0x401 (速度和高度資訊)
            # 計算速度：如果有最新速度資料就用，沒有就設為0
            if self.latest_velocity:
                velocity_magnitude = (self.latest_velocity.twist.linear.x**2 + 
                                    self.latest_velocity.twist.linear.y**2 + 
                                    self.latest_velocity.twist.linear.z**2)**0.5
                speed_kmh = velocity_magnitude * 3.6  # m/s to km/h
                speed_scaled = int(speed_kmh / 0.036)  # 根據Ecumaster格式
            else:
                speed_scaled = 0
            speed_bytes = struct.pack('<h', speed_scaled)
            
            # 高度：使用 NavSatFix 的 altitude
            height_scaled = int(msg.altitude) if not float('inf') == msg.altitude and not float('-inf') == msg.altitude else 0
            height_bytes = struct.pack('<h', height_scaled)
            
            # 衛星數量：NavSatFix 沒有此資料，設定預設值
            satellites_num = 8  # 預設值
            satellites_bytes = struct.pack('<B', satellites_num)
            
            # GPS frame index 和 status：預設值
            gps_frame_idx = 0
            empty_frame_idx = 0 
            gps_status = 1  # 假設GPS正常
            frame_status_byte = (gps_frame_idx & 0x0F) | ((empty_frame_idx & 0x0F) << 4)
            status_byte = gps_status & 0x07
            
            # 將 9 個 float64 (72 bytes) 以 9 個 CAN frame 傳送，每 frame 8 bytes
            cov_bytes = b''.join([struct.pack('<d', v) for v in msg.position_covariance])
            can_cov_msgs = []
            for i in range(9):
                frame_data = cov_bytes[i*8:(i+1)*8]  # 每個 covariance float64 拆成一個 frame
                can_cov_msgs.append(
                    can.Message(arbitration_id=0x410 + i, data=frame_data, is_extended_id=False)
                )

            # 新增第 10 個 frame：只送 type（1 byte）+ padding 7 byte
            frame_data = struct.pack('<B', msg.position_covariance_type) + b'\x00'*7
            can_cov_msgs.append(
                can.Message(arbitration_id=0x410 + 9, data=frame_data, is_extended_id=False)
            )
            for m in can_cov_msgs:
                self.bus.send(m)





            # 組合成8-byte資料，最後一個字節填0
            gps_extended_data = speed_bytes + height_bytes + satellites_bytes + struct.pack('<B', frame_status_byte) + struct.pack('<B', status_byte) + b'\x00'

            # 建立CAN訊息
            messages = [
                can.Message(arbitration_id=0x400, data=gps_basic_data, is_extended_id=False),      # GPS基本資訊
                can.Message(arbitration_id=0x401, data=gps_extended_data, is_extended_id=False),   # GPS擴展資訊
            ]

            for m in messages:
                self.bus.send(m)

            self.get_logger().info(
                f"Sent GPS CAN frames: "
                f"Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, "
                f"Alt={msg.altitude:.1f}m, Speed={speed_scaled*0.036:.1f}km/h"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send GPS CAN messages: {e}")
            
    def velocity_callback(self, msg: TwistStamped):
        try:
            # 儲存最新的速度資料，供GPS callback使用
            self.latest_velocity = msg
            
            # Ecumaster 格式編碼 - 0x402 (航向和角速度)
            # 航向資料：TwistStamped 沒有航向資訊，設定預設值
            heading_motion = 0  # 無航向資料
            heading_vehicle = 0  # 無航向資料
            heading_motion_bytes = struct.pack('<H', heading_motion)
            heading_vehicle_bytes = struct.pack('<H', heading_vehicle)
            
            # 角速度：使用 TwistStamped 的 angular 資料
            x_angle_rate_scaled = int(msg.twist.angular.x * 100)  # 轉換為 0.01°/s 單位
            y_angle_rate_scaled = int(msg.twist.angular.y * 100)
            x_angle_rate_bytes = struct.pack('<h', x_angle_rate_scaled)
            y_angle_rate_bytes = struct.pack('<h', y_angle_rate_scaled)
            
            # 組合成8-byte資料
            heading_data = heading_motion_bytes + heading_vehicle_bytes + x_angle_rate_bytes + y_angle_rate_bytes

            # Ecumaster 格式編碼 - 0x403 (Z角速度和加速度)
            # Z軸角速度：使用 TwistStamped 的 angular.z
            z_angle_rate_scaled = int(msg.twist.angular.z * 100)
            z_angle_rate_bytes = struct.pack('<h', z_angle_rate_scaled)
            
            # 加速度：TwistStamped 沒有加速度資料，設定為0
            # 實際上需要 IMU 資料才能取得加速度
            x_accel_scaled = 0
            y_accel_scaled = 0  
            z_accel_scaled = 0
            x_accel_bytes = struct.pack('<h', x_accel_scaled)
            y_accel_bytes = struct.pack('<h', y_accel_scaled)
            z_accel_bytes = struct.pack('<h', z_accel_scaled)
            
            # 組合成8-byte資料
            accel_data = z_angle_rate_bytes + x_accel_bytes + y_accel_bytes + z_accel_bytes

            # 額外編碼：各方向線性速度詳細資料 (參考原編碼方式，但使用Ecumaster ID範圍)
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
                # 標準Ecumaster格式
                can.Message(arbitration_id=0x402, data=heading_data, is_extended_id=False),     # 航向和角速度
                can.Message(arbitration_id=0x403, data=accel_data, is_extended_id=False),      # Z角速度和加速度
                
                # 擴展速度資料 (詳細各方向資料)
                can.Message(arbitration_id=0x404, data=vx_bytes, is_extended_id=False),        # X方向線性速度
                can.Message(arbitration_id=0x405, data=vy_bytes, is_extended_id=False),        # Y方向線性速度
                can.Message(arbitration_id=0x406, data=vz_bytes, is_extended_id=False),        # Z方向線性速度
                can.Message(arbitration_id=0x407, data=wx_bytes, is_extended_id=False),        # X軸角速度
                can.Message(arbitration_id=0x408, data=wy_bytes, is_extended_id=False),        # Y軸角速度
                can.Message(arbitration_id=0x409, data=wz_bytes, is_extended_id=False),        # Z軸角速度
                can.Message(arbitration_id=0x40A, data=vmag_bytes, is_extended_id=False),      # 總速度大小
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
