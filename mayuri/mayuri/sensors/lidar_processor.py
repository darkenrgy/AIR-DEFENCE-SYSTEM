import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
import numpy as np
import json
import math
import time
import threading


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Parameters
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('safety_radius', 3.0)
        self.declare_parameter('publish_rate', 5.0)

        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.safety_radius = float(self.get_parameter('safety_radius').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Internal state
        self.lock = threading.Lock()
        self.last_scan = None
        self.occupancy_grid = None

        # ROS2 Interfaces
        self.create_subscription(LaserScan, self.lidar_topic, self.on_scan_received, 10)
        self.map_pub = self.create_publisher(String, '/mayuri/sensor/obstacle_map', 10)
        self.status_pub = self.create_publisher(String, '/mayuri/lidar/status', 10)

        # Start background publishing thread
        self.running = True
        threading.Thread(target=self.publish_loop, daemon=True).start()

        self.get_logger().info(" LidarProcessor initialized and listening for scans.")

    # ----------------------------------------------------------------------
    def on_scan_received(self, scan: LaserScan):
        """Handle incoming LIDAR scan messages."""
        with self.lock:
            self.last_scan = scan
            self.occupancy_grid = self._generate_grid_from_scan(scan)
        self._publish_status("SCAN_RECEIVED")

    # ----------------------------------------------------------------------
    def _generate_grid_from_scan(self, scan):
        """Convert LIDAR scan data into a 2D occupancy grid."""
        angle = scan.angle_min
        grid_points = []

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            if self.min_range < r < self.max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                gx = int(x / self.grid_resolution)
                gy = int(y / self.grid_resolution)
                grid_points.append((gx, gy))
            angle += scan.angle_increment

        # Remove duplicates and nearby points (noise)
        grid_points = list(set(grid_points))

        # Build occupancy grid map (list of obstacle points)
        return grid_points

    # ----------------------------------------------------------------------
    def publish_loop(self):
        """Continuously publish obstacle map at given rate."""
        rate = 1.0 / max(0.01, self.publish_rate)
        while self.running and rclpy.ok():
            with self.lock:
                if self.occupancy_grid:
                    obstacles = self.occupancy_grid
                    msg_data = {
                        "timestamp": time.time(),
                        "obstacles": obstacles
                    }
                    msg = String()
                    msg.data = json.dumps(msg_data)
                    self.map_pub.publish(msg)
                    self._publish_status("MAP_PUBLISHED")
            time.sleep(rate)

    # ----------------------------------------------------------------------
    def _publish_status(self, status):
        """Publish LIDAR health/status messages."""
        msg = {
            "timestamp": time.time(),
            "status": status
        }
        s = String()
        s.data = json.dumps(msg)
        self.status_pub.publish(s)

    def destroy_node(self):
        self.running = False
        self.get_logger().info("LidarProcessor shutting down.")
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
