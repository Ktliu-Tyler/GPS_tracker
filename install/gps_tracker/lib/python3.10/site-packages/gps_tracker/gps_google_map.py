import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import contextily as ctx
import pyproj
import math

class GPSAnimateNode(Node):
    def __init__(self):
        super().__init__('gps_animate_node')
        self.fig, self.ax = plt.subplots()

        self.xs, self.ys = [], []  # 經度、緯度
        self.times = []
        self.mx, self.my = [], []  # Web Mercator
        self.line, = self.ax.plot([], [], 'b-', lw=2, zorder=2, label='Trajectory')
        self.current_point, = self.ax.plot([], [], 'ro', markersize=8, zorder=3)
        self.speed_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('GPS Trajectory Animation')
        plt.legend()

        # 訂閱 /fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=True)

        # 投影器：WGS84 -> Web Mercator
        self.proj = pyproj.Transformer.from_crs("epsg:4326", "epsg:3857", always_xy=True)

    def gps_callback(self, msg):
        if msg.status.status >= 0 and msg.latitude != 0.0 and msg.longitude != 0.0:
            self.ys.append(msg.latitude)
            self.xs.append(msg.longitude)
            mx, my = self.proj.transform(msg.longitude, msg.latitude)
            self.mx.append(mx)
            self.my.append(my)
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.times.append(now)
        self.get_logger().info(f'Received GPS data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')

    def update_plot(self, frame):
        if not self.mx or not self.my:
            return self.line, self.current_point, self.speed_text
        # 設定地圖顯示範圍（自動根據軌跡縮放）
        margin = 30  # 公尺
        minx, maxx = min(self.mx)-margin, max(self.mx)+margin
        miny, maxy = min(self.my)-margin, max(self.my)+margin
        self.ax.cla()
# 重新加標籤和底圖
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_title('GPS Trajectory Animation')
        self.ax.legend()
        self.ax.set_xlim(minx, maxx)
        self.ax.set_ylim(miny, maxy)
        ctx.add_basemap(self.ax, crs="epsg:3857", source=ctx.providers.OpenStreetMap.Mapnik, reset_extent=False)
        # 再畫軌跡
        self.ax.plot(self.mx, self.my, 'b-', lw=2, zorder=2, label='Trajectory')
        self.ax.plot([self.mx[-1]], [self.my[-1]], 'ro', markersize=8, zorder=3)
        self.line.set_data(self.mx, self.my)
        self.current_point.set_data([self.mx[-1]], [self.my[-1]])
        # 計算速度
        if len(self.mx) >= 2:
            dist = self.haversine(self.mx[-2], self.my[-2], self.mx[-1], self.my[-1])
            dt = self.times[-1] - self.times[-2]
            speed = dist / dt if dt > 0 else 0  # m/s
            self.speed_text.set_text(f"Speed: {speed:.2f} m/s")
        else:
            self.speed_text.set_text("Speed: N/A")
        return self.line, self.current_point, self.speed_text
    
    def haversine(self, x1, y1, x2, y2):
        # Web Mercator 平面距離（單位：公尺）
        return math.hypot(x2 - x1, y2 - y1)

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = GPSAnimateNode()
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()