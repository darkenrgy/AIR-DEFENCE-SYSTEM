import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time
import threading


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Parameters
        self.declare_parameter("safety_radius", 5.0)  # meters
        self.declare_parameter("avoidance_strength", 2.0)
        self.declare_parameter("update_rate", 5.0)

        self.safety_radius = float(self.get_parameter("safety_radius").value)
        self.avoidance_strength = float(self.get_parameter("avoidance_strength").value)
        self.update_rate = float(self.get_parameter("update_rate").value)

        # State
        self.obstacles = []  # [(x, y, dist), ...]
        self.current_path = []  # from path_planner
        self.safe_path = []
        self.lock = threading.Lock()

        # ROS2 Topics
        self.create_subscription(String, "/mayuri/sensor/obstacle_map", self.on_obstacle_update, 10)
        self.create_subscription(String, "/mayuri/path_planner/path", self.on_path_update, 10)
        self.safe_path_pub = self.create_publisher(String, "/mayuri/navigation/safe_path", 10)

        # Background thread for reactive path adjustment
        self.running = True
        self.worker = threading.Thread(target=self.avoidance_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(" ObstacleAvoidance module initialized.")

    # ---------------------------------------------------------------------
    def on_obstacle_update(self, msg: String):
        """Update obstacle data from sensors."""
        try:
            data = json.loads(msg.data)
            obs_list = data.get("obstacles", [])
            with self.lock:
                self.obstacles = [(float(o[0]), float(o[1])) for o in obs_list]
        except Exception as e:
            self.get_logger().warning(f"Obstacle update error: {e}")

    def on_path_update(self, msg: String):
        """Update latest path from path planner."""
        try:
            data = json.loads(msg.data)
            if not data.get("success"):
                return
            points = data.get("path", [])
            with self.lock:
                self.current_path = [(float(p["x"]), float(p["y"])) for p in points]
        except Exception as e:
            self.get_logger().warning(f"Path update error: {e}")

    # ---------------------------------------------------------------------
    def compute_avoidance_vector(self, drone_pos):
        """Compute repulsive vector from nearby obstacles."""
        ax, ay = 0.0, 0.0
        with self.lock:
            for ox, oy in self.obstacles:
                dx = drone_pos[0] - ox
                dy = drone_pos[1] - oy
                dist = math.sqrt(dx**2 + dy**2)
                if dist < self.safety_radius and dist > 0:
                    # Repulsive force (inverse-square)
                    strength = self.avoidance_strength / (dist**2)
                    ax += (dx / dist) * strength
                    ay += (dy / dist) * strength
        return ax, ay

    def adjust_path(self):
        """Modify current path with avoidance offsets."""
        with self.lock:
            if not self.current_path:
                return []

            adjusted_path = []
            for (x, y) in self.current_path:
                ax, ay = self.compute_avoidance_vector((x, y))
                new_x = x + ax
                new_y = y + ay
                adjusted_path.append((new_x, new_y))

            self.safe_path = adjusted_path
        return adjusted_path

    def publish_safe_path(self):
        """Publish the adjusted safe path."""
        msg = {
            "timestamp": time.time(),
            "safe_path": [{"x": p[0], "y": p[1]} for p in self.safe_path],
        }
        m = String()
        m.data = json.dumps(msg)
        self.safe_path_pub.publish(m)
        self.get_logger().info(f"==> Safe path updated ({len(self.safe_path)} points).")

    # ---------------------------------------------------------------------
    def avoidance_loop(self):
        """Continuous loop to recompute and publish adjusted paths."""
        rate = 1.0 / max(0.01, self.update_rate)
        while self.running and rclpy.ok():
            try:
                adjusted = self.adjust_path()
                if adjusted:
                    self.publish_safe_path()
            except Exception as e:
                self.get_logger().warning(f"Reactive avoidance error: {e}")
            time.sleep(rate)

    # ---------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()
        self.get_logger().info(" ObstacleAvoidance node stopped.")


# ---------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
