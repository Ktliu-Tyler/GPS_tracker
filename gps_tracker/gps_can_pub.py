import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import can
import struct

class CanPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10
        )
        self.get_logger().info("CAN Publisher Node Started.")

    def listener_callback(self, msg: NavSatFix):
        try:
            # 將 float 轉成 8-byte binary (double precision)
            lat_bytes = struct.pack('<d', msg.latitude)
            lon_bytes = struct.pack('<d', msg.longitude)
            alt_bytes = struct.pack('<d', msg.altitude)

            # 建立三筆 CAN 訊息，每筆長度 8 bytes
            messages = [
                can.Message(arbitration_id=0x130, data=lat_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x131, data=lon_bytes, is_extended_id=False),
                can.Message(arbitration_id=0x132, data=alt_bytes, is_extended_id=False)
            ]

            for m in messages:
                self.bus.send(m)

            self.get_logger().info(
                f"Sent CAN frames (double): "
                f"Lat={msg.latitude:.15f}, "
                f"Lon={msg.longitude:.15f}, "
                f"Alt={msg.altitude:.15f}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send CAN messages: {e}")

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
