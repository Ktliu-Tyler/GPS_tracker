#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import math
import time
from datetime import datetime

class AllSystemTestPublisher(Node):
    """發送所有系統假資料的測試發布器（僅ROS2版本）"""
    def __init__(self):
        super().__init__('all_system_test_publisher')
        
        # 建立ROS2發布者
        self.gps_publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/vel', 10)
        
        # 建立定時器
        self.timer_gps = self.create_timer(0.1, self.publish_gps_data)      # 10Hz GPS
        self.timer_vel = self.create_timer(0.02, self.publish_velocity_data) # 50Hz 速度
        
        # 模擬資料狀態
        self.time_counter = 0.0
        self.angle = 0.0
        self.speed_base = 15.0  # 基礎速度 m/s (約54km/h)
        
        # 台北101附近的測試路徑
        self.center_lat = 25.01365
        self.center_lon = 121.52861
        self.radius = 0.0003  # 較大的測試範圍
        
        self.get_logger().info('全系統測試發布器已啟動')
        self.get_logger().info('發送GPS和速度假資料到 /fix 和 /vel topics')
        self.get_logger().info('請同時執行 gps_can_pub_newEncode 來轉換為CAN訊號')
    
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
        msg.altitude = 32.5 + 8.0 * math.sin(self.angle * 0.5)  # 高度變化24.5-40.5m
        
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = 1
        
        self.gps_publisher.publish(msg)
        
        self.get_logger().info(
            f'GPS: 緯度:{msg.latitude:.6f}, 經度:{msg.longitude:.6f}, '
            f'高度:{msg.altitude:.1f}m, 角度:{math.degrees(self.angle):.1f}°'
        )
    
    def publish_velocity_data(self):
        """發布速度測試資料"""
        msg = TwistStamped()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 模擬真實車輛運動 - 圓形路徑的切線速度
        current_speed = self.speed_base + 5.0 * math.sin(self.time_counter * 0.3)  # 10-20 m/s
        
        # 切線方向的速度（垂直於半徑方向）
        tangent_angle = self.angle + math.pi/2
        msg.twist.linear.x = current_speed * math.cos(tangent_angle)
        msg.twist.linear.y = current_speed * math.sin(tangent_angle)
        msg.twist.linear.z = 0.2 * math.sin(self.time_counter * 2)  # 小幅度垂直運動
        
        # 模擬角速度 - 轉彎時的角速度
        angular_velocity = current_speed / (self.radius * 111320)  # 轉換為實際角速度
        msg.twist.angular.x = 0.1 * math.sin(self.time_counter * 1.5)  # 翻滾
        msg.twist.angular.y = 0.05 * math.cos(self.time_counter * 1.2)  # 俯仰
        msg.twist.angular.z = angular_velocity + 0.1 * math.sin(self.time_counter * 0.8)  # 偏航
        
        self.velocity_publisher.publish(msg)
        
        speed_kmh = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2) * 3.6
        self.get_logger().info(
            f'速度: {speed_kmh:.1f}km/h, 線性=({msg.twist.linear.x:.2f},{msg.twist.linear.y:.2f},{msg.twist.linear.z:.2f}), '
            f'角速度=({msg.twist.angular.x:.3f},{msg.twist.angular.y:.3f},{msg.twist.angular.z:.3f})'
        )
        
        # 更新計數器
        self.time_counter += 0.02
        self.angle += 0.005  # 較慢的圓周運動
        if self.angle >= 2 * math.pi:
            self.angle = 0.0

class HighFrequencyTestPublisher(Node):
    """高頻率測試發布器 - 用於性能測試"""
    def __init__(self):
        super().__init__('high_frequency_test_publisher')
        
        self.gps_publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/vel', 10)
        
        # 高頻率定時器
        self.timer_gps = self.create_timer(0.05, self.publish_gps_data)     # 20Hz GPS
        self.timer_vel = self.create_timer(0.01, self.publish_velocity_data) # 100Hz 速度
        
        self.time_counter = 0.0
        self.angle = 0.0
        
        self.get_logger().info('高頻率測試發布器已啟動 - GPS:20Hz, 速度:100Hz')
    
    def publish_gps_data(self):
        """發布高頻GPS資料"""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = 0
        msg.status.service = 1
        
        # 快速變化的路徑
        msg.latitude = 25.01365 + 0.0001 * math.sin(self.angle * 3)
        msg.longitude = 121.52861 + 0.0001 * math.cos(self.angle * 3)
        msg.altitude = 35.0 + 3.0 * math.sin(self.angle * 2)
        
        msg.position_covariance = [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]
        msg.position_covariance_type = 1
        
        self.gps_publisher.publish(msg)
    
    def publish_velocity_data(self):
        """發布高頻速度資料"""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 快速變化的速度模式
        speed = 25.0 + 10.0 * math.sin(self.time_counter * 2)
        msg.twist.linear.x = speed * math.cos(self.angle * 5)
        msg.twist.linear.y = speed * math.sin(self.angle * 5) * 0.4
        msg.twist.linear.z = 0.5 * math.sin(self.time_counter * 3)
        
        msg.twist.angular.x = 0.3 * math.sin(self.time_counter * 4)
        msg.twist.angular.y = 0.2 * math.cos(self.time_counter * 3)
        msg.twist.angular.z = 1.0 * math.sin(self.angle * 4)
        
        self.velocity_publisher.publish(msg)
        
        self.time_counter += 0.01
        self.angle += 0.02

class StaticTestPublisher(Node):
    """靜態測試發布器 - 用於基礎功能測試"""
    def __init__(self):
        super().__init__('static_test_publisher')
        
        self.gps_publisher = self.create_publisher(NavSatFix, '/fix', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/vel', 10)
        
        self.timer = self.create_timer(1.0, self.publish_static_data)  # 1Hz
        
        self.get_logger().info('靜態測試發布器已啟動 - 發送固定數值')
    
    def publish_static_data(self):
        """發布靜態測試資料"""
        # 固定GPS位置
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        gps_msg.status.status = 0
        gps_msg.status.service = 1
        gps_msg.latitude = 25.01365
        gps_msg.longitude = 121.52861
        gps_msg.altitude = 32.5
        gps_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        gps_msg.position_covariance_type = 1
        
        # 固定速度
        vel_msg = TwistStamped()
        vel_msg.header = Header()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'
        vel_msg.twist.linear.x = 10.0   # 10 m/s = 36 km/h
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0
        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = 0.1   # 0.1 rad/s 轉彎
        
        self.gps_publisher.publish(gps_msg)
        self.velocity_publisher.publish(vel_msg)
        
        self.get_logger().info(
            f'靜態資料: GPS=({gps_msg.latitude:.6f}, {gps_msg.longitude:.6f}), '
            f'速度=36km/h, 角速度=0.1rad/s'
        )

def main(args=None):
    rclpy.init(args=args)
    import sys
    
    # 根據命令列參數選擇不同的測試模式
    if len(sys.argv) > 1:
        if sys.argv[1] == 'all':
            node = AllSystemTestPublisher()
        elif sys.argv[1] == 'high':
            node = HighFrequencyTestPublisher()
        elif sys.argv[1] == 'static':
            node = StaticTestPublisher()
        else:
            print("🚗 GPS CAN測試發布器使用方式:")
            print("  ros2 run gps_tracker gps_test_publisher all    - 完整系統測試（推薦）")
            print("  ros2 run gps_tracker gps_test_publisher high   - 高頻率性能測試")
            print("  ros2 run gps_tracker gps_test_publisher static - 靜態基礎測試")
            print("")
            print("💡 測試步驟:")
            print("1. 啟動測試發布器: ros2 run gps_tracker gps_test_publisher all")
            print("2. 啟動CAN編碼器: ros2 run gps_tracker gps_can_pub_newEncode")
            print("3. 啟動CAN解碼器: ros2 run gps_tracker gps_can_rec")
            print("4. 觀察log確認資料正確傳輸")
            return
    else:
        print("請指定測試模式，使用 --help 查看使用方式")
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("測試發布器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
