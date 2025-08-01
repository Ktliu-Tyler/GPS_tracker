#!/usr/bin/env python3
"""
測試腳本：使用 CSV 文件來測試 CAN 解碼 dashboard
"""

import rclpy
from rclpy.node import Node
import time
import threading
from gps_tracker.gps_rec_with_csv import CanReceiver

def run_csv_dashboard():
    """運行 CSV dashboard 測試"""
    rclpy.init()
    
    # 創建參數覆蓋
    node = CanReceiver()
    
    # 手動設置參數
    node.use_csv = True
    node.csv_file = 'can_file.csv'  # 假設 CSV 文件在相同目錄
    node.csv_speed = 10.0  # 10倍速播放
    node.display_mode = 'dashboard'
    
    # 重新初始化 CSV 相關變量
    node.csv_data = []
    node.csv_index = 0
    node.csv_start_time = None
    node.load_csv_file()
    node.bus = None
    
    # 重新設置計時器
    node.timer.destroy()
    node.timer = node.create_timer(0.01, node.can_receive_callback)  # 10ms for CSV mode
    
    if node.display_mode == 'dashboard':
        if hasattr(node, 'display_timer'):
            node.display_timer.destroy()
        node.display_timer = node.create_timer(0.1, node.update_dashboard)  # 100ms
        import os
        os.system('clear')  # Clear screen initially
        node.get_logger().info("CSV Dashboard Test Started")
    
    print("Starting CSV dashboard test...")
    print("Press Ctrl+C to exit")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_csv_dashboard()
