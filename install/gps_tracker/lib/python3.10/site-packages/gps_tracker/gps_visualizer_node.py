import rclpy
from rclpy.node import Node
import matplotlib
matplotlib.use('Agg')  # 使用非GUI後端
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
import re
from datetime import datetime
import threading

class GPSVisualizerNode(Node):
    def __init__(self):
        super().__init__('gps_visualizer_node')
        
        # GPS資料儲存
        self.latitudes = []
        self.longitudes = []
        self.timestamps = []
        
        # 設定檔案路徑
        self.log_directory = os.path.expanduser('./gps_logs')
        today = datetime.now().strftime('%Y%m%d')
        self.log_file = os.path.join(self.log_directory, f'gps_log_{today}.txt')
        
        # 建立定時器，每秒更新一次資料
        self.timer = self.create_timer(2.0, self.update_data)
        
        # 設定圖片儲存路徑
        self.plot_directory = os.path.expanduser('./gps_plots')
        if not os.path.exists(self.plot_directory):
            os.makedirs(self.plot_directory)
        
        self.get_logger().info(f'GPS Visualizer Node 已啟動，讀取檔案: {self.log_file}')
        self.get_logger().info(f'圖片將儲存到: {self.plot_directory}')
        
    def parse_nmea_coordinate(self, coord_str, direction):
        """解析NMEA座標格式"""
        try:
            if len(coord_str) < 4:
                return None
                
            # 座標格式: DDMM.MMMMM (緯度) 或 DDDMM.MMMMM (經度)
            if direction in ['N', 'S']:  # 緯度
                degrees = int(coord_str[:2])
                minutes = float(coord_str[2:])
            else:  # 經度
                degrees = int(coord_str[:3])
                minutes = float(coord_str[3:])
            
            decimal_degrees = degrees + minutes / 60.0
            
            # 南緯和西經為負值
            if direction in ['S', 'W']:
                decimal_degrees = -decimal_degrees
                
            return decimal_degrees
        except:
            return None
    
    def update_data(self):
        """從檔案讀取最新的GPS資料並生成圖片"""
        if not os.path.exists(self.log_file):
            return
            
        try:
            with open(self.log_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # 清空舊資料
            new_latitudes = []
            new_longitudes = []
            new_timestamps = []
            
            for line in lines:
                line = line.strip()
                if line.startswith('$GNRMC') or line.startswith('$GNGGA'):
                    parts = line.split(',')
                    
                    if line.startswith('$GNRMC') and len(parts) >= 10:
                        # GNRMC格式: $GNRMC,時間,狀態,緯度,N/S,經度,E/W,速度,航向,日期...
                        if parts[2] == 'A':  # 有效資料
                            lat = self.parse_nmea_coordinate(parts[3], parts[4])
                            lon = self.parse_nmea_coordinate(parts[5], parts[6])
                            timestamp = parts[1]
                            
                            if lat is not None and lon is not None:
                                new_latitudes.append(lat)
                                new_longitudes.append(lon)
                                new_timestamps.append(timestamp)
                    
                    elif line.startswith('$GNGGA') and len(parts) >= 15:
                        # GNGGA格式: $GNGGA,時間,緯度,N/S,經度,E/W,品質,衛星數,HDOP,高度...
                        if parts[6] != '0':  # 有GPS fix
                            lat = self.parse_nmea_coordinate(parts[2], parts[3])
                            lon = self.parse_nmea_coordinate(parts[4], parts[5])
                            timestamp = parts[1]
                            
                            if lat is not None and lon is not None:
                                new_latitudes.append(lat)
                                new_longitudes.append(lon)
                                new_timestamps.append(timestamp)
            
            # 更新資料
            if new_latitudes and new_longitudes:
                self.latitudes = new_latitudes
                self.longitudes = new_longitudes
                self.timestamps = new_timestamps
                
                # 生成並儲存圖片
                self.generate_plot()
                
                self.get_logger().info(f'更新GPS資料，共 {len(self.latitudes)} 個點')
                
        except Exception as e:
            self.get_logger().error(f'讀取GPS檔案時發生錯誤: {str(e)}')
    
    def generate_plot(self):
        """生成GPS軌跡圖並儲存"""
        if not self.latitudes or not self.longitudes:
            return
        
        try:
            # 建立新的圖片
            plt.figure(figsize=(12, 8))
            
            # 繪製軌跡線
            plt.plot(self.longitudes, self.latitudes, 'b-', linewidth=2, label='GPS trackline', alpha=0.7)
            
            # 標記起點
            if len(self.latitudes) > 0:
                plt.plot(self.longitudes[0], self.latitudes[0], 'go', markersize=10, label='start')
            
            # 標記終點/目前位置
            if len(self.latitudes) > 1:
                plt.plot(self.longitudes[-1], self.latitudes[-1], 'ro', markersize=10, label='end')
            
            # 設定圖表
            plt.xlabel('(Longitude)')
            plt.ylabel('(Latitude)')
            plt.title(f'GPS_trackline - {datetime.now().strftime("%Y-%m-%d %H:%M:%S")} ({len(self.latitudes)} 個點)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # 自動調整視圖範圍
            if len(self.latitudes) > 1:
                lat_margin = (max(self.latitudes) - min(self.latitudes)) * 0.1 + 0.001
                lon_margin = (max(self.longitudes) - min(self.longitudes)) * 0.1 + 0.001
                
                plt.xlim(min(self.longitudes) - lon_margin, max(self.longitudes) + lon_margin)
                plt.ylim(min(self.latitudes) - lat_margin, max(self.latitudes) + lat_margin)
            
            # 儲存圖片
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            plot_filename = os.path.join(self.plot_directory, f'gps_track_{timestamp}.png')
            plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
            
            
            latest_filename = os.path.join(self.plot_directory, 'gps_track_latest.png')
            plt.savefig(latest_filename, dpi=150, bbox_inches='tight')
            
            plt.close() 
            
            self.get_logger().info(f'已儲存GPS軌跡圖: {plot_filename}')
            
        except Exception as e:
            self.get_logger().error(f'生成GPS圖片時發生錯誤: {str(e)}')
    
    def animate(self, frame):
        pass
    
    def show_plot(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    
    gps_visualizer = None
    try:
        gps_visualizer = GPSVisualizerNode()
        rclpy.spin(gps_visualizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if gps_visualizer:
            gps_visualizer.get_logger().error(f'發生錯誤: {str(e)}')
    finally:
        if gps_visualizer:
            gps_visualizer.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()