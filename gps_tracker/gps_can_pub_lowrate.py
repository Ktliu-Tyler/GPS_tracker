#!/usr/bin/env python3
"""
超低頻率 CAN Publisher (針對高流量 SPI CAN)
- 極低發送頻率 (1Hz)
- 只發送最關鍵資料
- 使用優先級更高的 ID
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray, String
import can
import struct
import time
import threading
from datetime import datetime

class CanPublisherLowRate(Node):
    def __init__(self):
        super().__init__('gps_can_pub_node')
        
        # CAN Bus 初始化
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # 針對 MCP251x SPI CAN 的極端優化配置
        self.can_send_delay = 0.05   # 每個 frame 之間 50ms
        self.can_retry_count = 2     # 只重試 2 次
        self.can_retry_delay = 0.1   # 重試間隔 100ms
        
        # 極低發送頻率 (適應高流量環境)
        self.last_gps_send_time = 0.0
        self.last_vel_send_time = 0.0
        self.last_timestamp_send_time = 0.0
        self.gps_send_interval = 1.0   # GPS 1Hz
        self.vel_send_interval = 1.0   # Velocity 1Hz (與 GPS 錯開)
        self.timestamp_send_interval = 1.0  # Timestamp 1Hz
        
        # 統計資訊
        self.send_success_count = 0
        self.send_failure_count = 0
        self.rx_cleared_count = 0
        
        # 緩衝區清理執行緒
        self.buffer_clear_running = True
        self.buffer_clear_thread = threading.Thread(target=self.clear_rx_buffer_loop, daemon=True)
        self.buffer_clear_thread.start()
        
        self.get_logger().info("Started RX buffer clearing thread")
        
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
        
        # 建立時間戳發布者
        self.timestamp_publisher = self.create_publisher(
            UInt8MultiArray, 
            'can_timestamp', 
            10
        )
        
        # 建立時間戳定時器
        self.timestamp_timer = self.create_timer(1.0, self.send_timestamp)
        
        # 統計定時器 (每 10 秒顯示一次)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        
        self.get_logger().info("CAN Publisher Low-Rate Mode Started")
        self.get_logger().info("Optimized for high-traffic SPI CAN (MCP251x)")
        self.get_logger().info("Configuration:")
        self.get_logger().info(f"  GPS rate: 1 Hz")
        self.get_logger().info(f"  Velocity rate: 1 Hz")
        self.get_logger().info(f"  Frame delay: 50 ms")
    
    def send_can_message_safe(self, msg, retry_count=None):
        """安全發送 CAN (針對高流量環境優化)"""
        if retry_count is None:
            retry_count = self.can_retry_count
        
        for attempt in range(retry_count):
            try:
                # 使用較長 timeout
                self.bus.send(msg, timeout=0.3)
                self.send_success_count += 1
                return True
            except can.CanError as e:
                if "Transmit buffer full" in str(e):
                    if attempt < retry_count - 1:
                        # 嘗試清空接收緩衝區
                        try:
                            for _ in range(10):
                                if self.bus.recv(timeout=0) is None:
                                    break
                        except:
                            pass
                        time.sleep(self.can_retry_delay)
                        continue
                    else:
                        self.send_failure_count += 1
                        return False
                else:
                    self.get_logger().debug(f'CAN error: {e}')
                    self.send_failure_count += 1
                    return False
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
                self.send_failure_count += 1
                return False
        return False
    
    def clear_rx_buffer_loop(self):
        """後台執行緒：持續清空接收緩衝區"""
        while self.buffer_clear_running:
            try:
                # 持續讀取並丟棄接收的 CAN 訊息
                cleared_in_batch = 0
                while True:
                    msg = self.bus.recv(timeout=0.001)  # 1ms timeout
                    if msg is None:
                        break
                    cleared_in_batch += 1
                    self.rx_cleared_count += 1
                    
                    # 批次達到 100 時記錄
                    if cleared_in_batch >= 100:
                        self.get_logger().debug(f"Cleared {cleared_in_batch} RX messages")
                        cleared_in_batch = 0
                
                # 短暫休息避免 CPU 100%
                time.sleep(0.01)  # 10ms
                
            except Exception as e:
                self.get_logger().debug(f"RX buffer clear error: {e}")
                time.sleep(0.1)
    
    def print_stats(self):
        """顯示統計資訊"""
        total = self.send_success_count + self.send_failure_count
        if total > 0:
            success_rate = (self.send_success_count / total) * 100
            self.get_logger().info(
                f"CAN Stats: TX Success={self.send_success_count}, "
                f"TX Failed={self.send_failure_count}, "
                f"TX Rate={success_rate:.1f}%, "
                f"RX Cleared={self.rx_cleared_count}"
            )
    
    def send_timestamp(self):
        """發送時間戳 (使用高優先級 ID)"""
        current_time = time.time()
        if current_time - self.last_timestamp_send_time < self.timestamp_send_interval:
            return
        
        try:
            now = time.time()
            ms_since_midnight = int((now % 86400) * 1000)
            days_since_1984 = int((now - 441763200) // 86400)
            
            timestamp_data = ms_since_midnight.to_bytes(4, 'little') + days_since_1984.to_bytes(2, 'little')
            
            # 使用較高優先級 ID (0x100)
            can_msg = can.Message(
                arbitration_id=0x100, 
                data=timestamp_data, 
                is_extended_id=False
            )
            
            if self.send_can_message_safe(can_msg):
                self.last_timestamp_send_time = current_time
                
                ros_msg = UInt8MultiArray()
                ros_msg.data = list(timestamp_data)
                self.timestamp_publisher.publish(ros_msg)
                
                self.get_logger().debug(f'Timestamp sent: {ms_since_midnight}ms')
            
        except Exception as e:
            self.get_logger().error(f'Timestamp error: {e}')

    def gps_callback(self, msg: NavSatFix):
        """GPS 位置 - 只發送 1 個 frame"""
        current_time = time.time()
        
        # 頻率限制
        if current_time - self.last_gps_send_time < self.gps_send_interval:
            return
        
        # 與 velocity 錯開 500ms
        if abs(current_time - self.last_vel_send_time) < 0.5:
            return
        
        try:
            # 使用較低精度避免溢出 (4 位小數點足夠)
            lat_scaled = int(msg.latitude * 10**4)   # 4 bytes
            lon_scaled = int(msg.longitude * 10**4)  # 4 bytes
            
            # 緯度: 4 bytes (signed), 經度: 4 bytes (signed)
            # 只能放 8 bytes，所以不包含高度
            gps_data = (
                lat_scaled.to_bytes(4, 'little', signed=True) +
                lon_scaled.to_bytes(4, 'little', signed=True)
            )
            
            can_msg = can.Message(
                arbitration_id=0x200,  # 較高優先級
                data=gps_data,
                is_extended_id=False
            )
            
            if self.send_can_message_safe(can_msg):
                self.last_gps_send_time = current_time
                self.get_logger().info(
                    f"GPS: Lat={msg.latitude:.4f}, Lon={msg.longitude:.4f}, Alt={msg.altitude:.0f}m"
                )
            
        except Exception as e:
            self.get_logger().error(f"GPS error: {e}")
            
    def velocity_callback(self, msg: TwistStamped):
        """速度 - 只發送 1 個 frame"""
        current_time = time.time()
        
        # 頻率限制
        if current_time - self.last_vel_send_time < self.vel_send_interval:
            return
        
        # 與 GPS 錯開 500ms
        if abs(current_time - self.last_gps_send_time) < 0.5:
            return
        
        try:
            # 計算總速度
            velocity_magnitude = (msg.twist.linear.x**2 + msg.twist.linear.y**2 + msg.twist.linear.z**2)**0.5
            
            # 速度資料: VX, VY (各 2 bytes), 總速度 (2 bytes)
            vx_scaled = int(msg.twist.linear.x * 100)  # cm/s
            vy_scaled = int(msg.twist.linear.y * 100)
            vmag_scaled = int(velocity_magnitude * 100)
            
            vel_data = (
                vx_scaled.to_bytes(2, 'little', signed=True) +
                vy_scaled.to_bytes(2, 'little', signed=True) +
                vmag_scaled.to_bytes(2, 'little', signed=True) +
                b'\x00\x00'  # 填充
            )
            
            can_msg = can.Message(
                arbitration_id=0x201,  # 較高優先級
                data=vel_data,
                is_extended_id=False
            )
            
            if self.send_can_message_safe(can_msg):
                self.last_vel_send_time = current_time
                speed_kmh = velocity_magnitude * 3.6
                self.get_logger().info(f"Velocity: Speed={speed_kmh:.2f}km/h")
            
        except Exception as e:
            self.get_logger().error(f"Velocity error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanPublisherLowRate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        # 停止緩衝區清理執行緒
        node.buffer_clear_running = False
        if node.buffer_clear_thread.is_alive():
            node.buffer_clear_thread.join(timeout=1.0)
        
        # 顯示最終統計
        node.print_stats()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
