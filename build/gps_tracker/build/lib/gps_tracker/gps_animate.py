import rclpy
from rclpy.node import Node
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import math

class GPSAnimateNode(Node):
    def __init__(self):
        super().__init__('gps_animate_node')
        self.file_path = '/home/docker/gps_logs/gps_6114'
        self.gps_data, self.time_stamps = self.parse_gps_file(self.file_path)
        self.fig, self.ax = plt.subplots()

        # 讀取衛星雲圖
        self.bg_img = mpimg.imread('/home/docker/ws/src/nturt_ros/gps_tracker/gps_tracker/image2.png')
        self.bg_extent = [121.527909-0.00005, 121.528739-0.00005, 25.013220-0.00005, 25.013831-0.00005] 
        self.ax.imshow(self.bg_img, extent=self.bg_extent, aspect='auto', zorder=0)

        self.line, = self.ax.plot([], [], 'b-', lw=2, zorder=2)
        self.current_point, = self.ax.plot([], [], 'ro', markersize=8, zorder=3)
        self.speed_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        self.xs, self.ys = [], []
        self.ani = animation.FuncAnimation(
            self.fig, self.update, frames=len(self.gps_data),
            init_func=self.init_plot, interval=100, blit=True, repeat=False)
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('GPS Trajectory Animation')
        plt.show()

    def init_plot(self):
        self.ax.set_xlim(self.bg_extent[0], self.bg_extent[1])
        self.ax.set_ylim(self.bg_extent[2], self.bg_extent[3])
        self.line.set_data([], [])
        self.current_point.set_data([], [])
        self.speed_text.set_text('')
        return self.line, self.current_point, self.speed_text

    def update(self, frame):
        lat, lon, _ = self.gps_data[frame]
        self.xs.append(lon)
        self.ys.append(lat)
        self.line.set_data(self.xs, self.ys)
        self.current_point.set_data([lon], [lat])
        # 計算速度
        if frame > 0:
            prev_lat, prev_lon, _ = self.gps_data[frame-1]
            prev_time = self.time_stamps[frame-1]
            curr_time = self.time_stamps[frame]
            dist = self.haversine(prev_lon, prev_lat, lon, lat)  # 單位: 公尺
            dt = curr_time - prev_time
            if dt > 0:
                speed = dist / dt  # m/s
                self.speed_text.set_text(f"Speed: {speed:.2f} m/s")
            else:
                self.speed_text.set_text("Speed: N/A")
        else:
            self.speed_text.set_text("Speed: N/A")
        return self.line, self.current_point, self.speed_text

    def parse_gps_file(self, file_path):
        gps_points = []
        time_stamps = []
        with open(file_path, 'r') as f:
            lines = f.readlines()
        lat, lon, speed = None, None, 0.0
        curr_time = 0.0
        for idx, line in enumerate(lines):
            # 假設每行資料間隔固定，例如1秒
            curr_time = idx  # 若有時間戳請自行解析
            if line.startswith('$GNGGA'):
                parts = line.strip().split(',')
                if len(parts) > 5 and parts[2] and parts[4]:
                    lat = self.nmea_to_decimal(parts[2], parts[3])
                    lon = self.nmea_to_decimal(parts[4], parts[5])
            elif line.startswith('$GNRMC'):
                parts = line.strip().split(',')
                if len(parts) > 7 and parts[3] and parts[5]:
                    lat = self.nmea_to_decimal(parts[3], parts[4])
                    lon = self.nmea_to_decimal(parts[5], parts[6])
            if lat is not None and lon is not None:
                gps_points.append((lat, lon, 0.0))
                time_stamps.append(curr_time)
        return gps_points, time_stamps

    def nmea_to_decimal(self, value, direction):
        if not value or value == 'nan':
            return None
        d, m = re.match(r"(\d+)(\d\d\.\d+)", value).groups()
        decimal = float(d) + float(m) / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def haversine(self, lon1, lat1, lon2, lat2):
        # 經緯度轉弧度
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # 地球半徑（公尺）
        return c * r

def main(args=None):
    rclpy.init(args=args)
    node = GPSAnimateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()