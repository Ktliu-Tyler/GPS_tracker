#!/usr/bin/env python3
import can
import rclpy
from rclpy.node import Node
from datetime import datetime

class TimerPublisherNode(Node):
    def __init__(self):
        super().__init__('time_publisher_node')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.timer = self.create_timer(1.0, self.send_time_frame)

    def send_time_frame(self):
        now = datetime.now()

        year = now.year % 100         # 取兩位數
        month = now.month
        date = now.day
        hour = now.hour
        minute = now.minute
        second = now.second
        second40 = int(now.microsecond / 40000)  # 每 40ms 為一單位

        data = [
            year,
            month,
            date,
            hour,
            minute,
            second,
            second40
        ]

        msg = can.Message(
            arbitration_id=0x150,
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(msg)
            self.get_logger().info(f'Sent time CAN frame: {data}')
        except can.CanError as e:
            self.get_logger().error(f'CAN transmission failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TimerPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
