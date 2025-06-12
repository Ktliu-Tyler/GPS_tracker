#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from datetime import datetime
import os
import re

class GPSLoggerNode(Node):
    def __init__(self):
        super().__init__('gps_logger_node')
        
        # 建立訂閱者，訂閱 /fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # 設定儲存檔案的路徑
        self.log_directory = os.path.expanduser('./gps_logs')
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)
            
        # 建立今天的日誌檔案
        today = datetime.now().strftime('%Y%m%d')
        self.log_file = os.path.join(self.log_directory, f'gps_log_{today}.txt')
        
        self.get_logger().info(f'GPS Logger Node 已啟動，將記錄到: {self.log_file}')
        
    def gps_callback(self, msg):
        """處理接收到的GPS資料"""
        try:
            # 檢查GPS資料是否有效
            if msg.status.status >= 0:  # GPS fix available
                # 取得時間戳記
                timestamp = datetime.now().strftime('%H%M%S.%f')[:-3]  # HHMMSS.mmm格式
                current_date = datetime.now().strftime('%d%m%y')  # DDMMYY格式
                
                # 轉換座標格式 (度分格式)
                lat_deg = int(abs(msg.latitude))
                lat_min = (abs(msg.latitude) - lat_deg) * 60
                lat_dir = 'N' if msg.latitude >= 0 else 'S'
                
                lon_deg = int(abs(msg.longitude))
                lon_min = (abs(msg.longitude) - lon_deg) * 60
                lon_dir = 'E' if msg.longitude >= 0 else 'W'
                
                # 格式化座標
                lat_str = f"{lat_deg:02d}{lat_min:08.5f}"
                lon_str = f"{lon_deg:03d}{lon_min:08.5f}"
                
                # 建立NMEA格式的GNRMC資料
                gnrmc_line = f"$GNRMC,{timestamp},A,{lat_str},{lat_dir},{lon_str},{lon_dir},0.004,0.00,{current_date},,,A*00"
                
                # 建立NMEA格式的GNGGA資料
                gngga_line = f"$GNGGA,{timestamp},{lat_str},{lat_dir},{lon_str},{lon_dir},1,08,1.0,{msg.altitude:.1f},M,0.0,M,,*00"
                
                # 寫入檔案
                with open(self.log_file, 'a', encoding='utf-8') as f:
                    f.write(f"{gnrmc_line}\n")
                    f.write(f"{gngga_line}\n")
                
                self.get_logger().info(f'已記錄GPS資料: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}')
                
            else:
                self.get_logger().warn('GPS訊號無效，跳過記錄')
                
        except Exception as e:
            self.get_logger().error(f'處理GPS資料時發生錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gps_logger = GPSLoggerNode()
        rclpy.spin(gps_logger)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gps_logger' in locals():
            gps_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()