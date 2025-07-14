import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
import can
import struct
from datetime import datetime

class CanPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
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
        
        self.get_logger().info("CAN Publisher Node Started.")

    def gps_callback(self, msg: NavSatFix):
        try:
            # 將 GPS 位置資料轉成 8-byte binary (double precision)
            lat_bytes = struct.pack('<d', msg.latitude)
            lon_bytes = struct.pack('<d', msg.longitude)
            alt_bytes = struct.pack('<d', msg.altitude)
            
            # 記錄當前時間 (格式: 20250712225600)
            current_time_num = float(datetime.now().strftime("%Y%m%d%H%M%S"))
            time_bytes = struct.pack('<d', current_time_num)

            # 建立GPS和時間的CAN訊息
            messages = [
                can.Message(arbitration_id=0x130, data=lat_bytes, is_extended_id=False),    # 緯度
                can.Message(arbitration_id=0x131, data=lon_bytes, is_extended_id=False),    # 經度
                can.Message(arbitration_id=0x132, data=alt_bytes, is_extended_id=False),    # 高度
                can.Message(arbitration_id=0x140, data=time_bytes, is_extended_id=False)    # 時間戳記
            ]

            for m in messages:
                self.bus.send(m)

            self.get_logger().info(
                f"Sent GPS CAN frames: "
                f"Lat={msg.latitude:.6f}, "
                f"Lon={msg.longitude:.6f}, "
                f"Alt={msg.altitude:.1f}, "
                f"Time={current_time_num:.0f}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send GPS CAN messages: {e}")
            
    def velocity_callback(self, msg: TwistStamped):
        try:
            # 將速度和角速度資料轉成 8-byte binary (double precision)
            vx_bytes = struct.pack('<d', msg.twist.linear.x)   # X方向速度
            vy_bytes = struct.pack('<d', msg.twist.linear.y)   # Y方向速度  
            vz_bytes = struct.pack('<d', msg.twist.linear.z)   # Z方向速度
            
            wx_bytes = struct.pack('<d', msg.twist.angular.x)  # X軸角速度
            wy_bytes = struct.pack('<d', msg.twist.angular.y)  # Y軸角速度
            wz_bytes = struct.pack('<d', msg.twist.angular.z)  # Z軸角速度
            
            # 計算總速度大小
            velocity_magnitude = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
            vmag_bytes = struct.pack('<d', velocity_magnitude)

            # 建立速度和角速度的CAN訊息
            messages = [
                can.Message(arbitration_id=0x133, data=vx_bytes, is_extended_id=False),    # X方向速度
                can.Message(arbitration_id=0x134, data=vy_bytes, is_extended_id=False),    # Y方向速度
                can.Message(arbitration_id=0x135, data=vz_bytes, is_extended_id=False),    # Z方向速度
                can.Message(arbitration_id=0x136, data=wx_bytes, is_extended_id=False),    # X軸角速度
                can.Message(arbitration_id=0x137, data=wy_bytes, is_extended_id=False),    # Y軸角速度
                can.Message(arbitration_id=0x138, data=wz_bytes, is_extended_id=False),    # Z軸角速度
                can.Message(arbitration_id=0x139, data=vmag_bytes, is_extended_id=False)   # 總速度大小
            ]

            for m in messages:
                self.bus.send(m)

            self.get_logger().info(
                f"Sent Velocity CAN frames: "
                f"Linear=({msg.twist.linear.x:.6f}, {msg.twist.linear.y:.6f}, {msg.twist.linear.z:.6f}), "
                f"Angular=({msg.twist.angular.x:.6f}, {msg.twist.angular.y:.6f}, {msg.twist.angular.z:.6f}), "
                f"Magnitude={velocity_magnitude:.6f}"
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
