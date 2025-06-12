import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import can
import struct

class GpsCanBridge(Node):
    def __init__(self):
        super().__init__('gps_can_bridge')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 初始化CAN接口
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def listener_callback(self, msg: NavSatFix):
        # 將GPS資料轉換為CAN訊息
        # 這裡以經緯度各4字節(float32)為例，總共8字節
        lat_bytes = struct.pack('<f', msg.latitude)
        lon_bytes = struct.pack('<f', msg.longitude)
        data = lat_bytes + lon_bytes  # 8 bytes

        can_msg = can.Message(arbitration_id=0x123, data=data, is_extended_id=False)
        try:
            self.bus.send(can_msg)
            self.get_logger().info(f'Sent CAN msg: lat={msg.latitude}, lon={msg.longitude}')
        except can.CanError:
            self.get_logger().error('CAN message NOT sent')

def main(args=None):
    rclpy.init(args=args)
    node = GpsCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()