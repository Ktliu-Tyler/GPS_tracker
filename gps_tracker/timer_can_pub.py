import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import can
import time


class CANTimestampNode(Node):
    """ROS2 Node: Periodically sends CAN timestamp messages"""
    
    def __init__(self):
        super().__init__('can_timestamp_node')
        
        # Declare parameters
        self.declare_parameter('interval', 1.0)  # Send interval in seconds
        self.declare_parameter('can_channel', 'can0')  # CAN channel
        self.declare_parameter('can_id', 0x100)  # CAN ID
        
        # Get parameters
        self.interval = self.get_parameter('interval').get_parameter_value().double_value
        can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        
        # Initialize CAN interface
        try:
            self.can_bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            self.get_logger().info(f'CAN interface initialized successfully: {can_channel}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN interface: {e}')
            self.can_bus = None
        
        # Create publisher for ROS2 messages
        self.timestamp_publisher = self.create_publisher(
            UInt8MultiArray, 
            'can_timestamp', 
            10
        )
        
        # Create timer
        self.timer = self.create_timer(self.interval, self.send_timestamp)
        
        self.get_logger().info(f'CAN timestamp node started, interval: {self.interval} seconds')
    
    def send_timestamp(self):
        """Callback function to send timestamp"""
        try:
            # Calculate timestamp
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            # Prepare data (little endian)
            data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little')
            
            # Send CAN message
            if self.can_bus:
                can_msg = can.Message(
                    arbitration_id=self.can_id, 
                    data=data, 
                    is_extended_id=False
                )
                self.can_bus.send(can_msg)
            
            # Publish ROS2 message
            ros_msg = UInt8MultiArray()
            ros_msg.data = list(data)
            self.timestamp_publisher.publish(ros_msg)
            
            # Log information
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            self.get_logger().info(
                f'Timestamp sent - Time: {current_time}, '
                f'MS since midnight: {ms_since_midnight}, '
                f'Days since 1984: {days_since_1984}'
            )
            
        except can.CanError as e:
            self.get_logger().error(f'CAN Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending timestamp: {e}')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.can_bus:
            self.can_bus.shutdown()
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = CANTimestampNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()