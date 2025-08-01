import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import math
import time

class GPSAnimateNode(Node):
    def __init__(self):
        super().__init__('gps_animate_node')
        self.fig, self.ax = plt.subplots()
        self.bg_img = mpimg.imread('/home/docker/ws/src/nturt_ros/gps_tracker/gps_tracker/image2.png')
        self.bg_extent = [121.527909-0.00005, 121.528739-0.00005, 25.013220-0.00005, 25.013831-0.00005]
        self.ax.imshow(self.bg_img, extent=self.bg_extent, aspect='auto', zorder=0)

        self.xs, self.ys = [], []
        self.times = []  # 新增：記錄每個點的時間
        self.line, = self.ax.plot([], [], 'b-', lw=2, zorder=2, label='Trajectory')
        self.current_point, = self.ax.plot([], [], 'ro', markersize=8, zorder=3)
        self.speed_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        self.ax.set_xlim(self.bg_extent[0], self.bg_extent[1])
        self.ax.set_ylim(self.bg_extent[2], self.bg_extent[3])
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('GPS Trajectory Animation')
        plt.legend()

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=True)

    def gps_callback(self, msg):
        if msg.status.status >= 0 and msg.latitude != 0.0 and msg.longitude != 0.0:
            self.ys.append(msg.latitude)
            self.xs.append(msg.longitude)
            # 儲存時間（用ROS時間或系統時間都可）
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.times.append(now)
        self.get_logger().info(f'Received GPS data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')

    def update_plot(self, frame):
        self.line.set_data(self.xs, self.ys)
        if self.xs and self.ys:
            self.current_point.set_data([self.xs[-1]], [self.ys[-1]])
            # 計算速度
            if len(self.xs) >= 2:
                dist = self.haversine(self.xs[-2], self.ys[-2], self.xs[-1], self.ys[-1])
                dt = self.times[-1] - self.times[-2]
                speed = dist / dt if dt > 0 else 0  # 單位：公尺/秒
                self.speed_text.set_text(f"Speed: {speed:.2f} m/s")
            else:
                self.speed_text.set_text("Speed: N/A")
        else:
            self.current_point.set_data([], [])
            self.speed_text.set_text("Speed: N/A")
        return self.line, self.current_point, self.speed_text

    def haversine(self, lon1, lat1, lon2, lat2):
        # 經緯度轉弧度
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # 地球半徑（公尺）
        return c * r

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