import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time
import random
import threading


class RadarProcessor(Node):
    def __init__(self):
        super().__init__("radar_processor")

        # Parameters
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("radar_port", "/dev/ttyUSB0")
        self.declare_parameter("scan_frequency", 5.0)
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("target_threshold", 0.5)

        self.simulation_mode = bool(self.get_parameter("simulation_mode").value)
        self.radar_port = self.get_parameter("radar_port").value
        self.scan_frequency = float(self.get_parameter("scan_frequency").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.target_threshold = float(self.get_parameter("target_threshold").value)

        # Publishers
        self.data_pub = self.create_publisher(String, "/mayuri/sensor/radar_data", 10)
        self.status_pub = self.create_publisher(String, "/mayuri/radar/status", 10)

        # Internal state
        self.running = True
        self.lock = threading.Lock()

        # Background thread for radar updates
        self.thread = threading.Thread(target=self._radar_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(" RadarProcessor initialized and scanning.")

    # ----------------------------------------------------------------------
    def _simulate_radar_readings(self):
        """Simulate radar detections for testing."""
        detections = []
        for _ in range(random.randint(1, 5)):
            distance = round(random.uniform(self.min_range, self.max_range), 2)
            angle = round(random.uniform(-math.pi, math.pi), 2)
            velocity = round(random.uniform(-10, 10), 2)  # m/s
            signal_strength = round(random.uniform(0.5, 1.0), 2)
            if signal_strength > self.target_threshold:
                detections.append({
                    "distance": distance,
                    "angle": angle,
                    "velocity": velocity,
                    "signal_strength": signal_strength
                })
        return detections

    # ----------------------------------------------------------------------
    def _parse_hardware_data(self):
        """
        Placeholder for hardware radar driver.
        In production, this function reads serial data packets from radar.
        """
        # Example: read binary data, decode, and extract detections
        # For now, simulate hardware behavior
        return self._simulate_radar_readings()

    # ----------------------------------------------------------------------
    def _radar_loop(self):
        """Continuously collect and publish radar data."""
        rate = 1.0 / max(0.01, self.scan_frequency)
        while self.running and rclpy.ok():
            try:
                detections = (self._simulate_radar_readings()
                              if self.simulation_mode else self._parse_hardware_data())

                msg_data = {
                    "timestamp": time.time(),
                    "detections": detections
                }

                msg = String()
                msg.data = json.dumps(msg_data)
                self.data_pub.publish(msg)
                self._publish_status("RADAR_SCAN_OK")

                self.get_logger().debug(f"Radar scan: {len(detections)} detections")

            except Exception as e:
                self.get_logger().warning(f"Radar loop error: {e}")
                self._publish_status("RADAR_ERROR")

            time.sleep(rate)

    # ----------------------------------------------------------------------
    def _publish_status(self, status):
        """Publish radar health/status info."""
        msg = {
            "timestamp": time.time(),
            "status": status
        }
        s = String()
        s.data = json.dumps(msg)
        self.status_pub.publish(s)

    # ----------------------------------------------------------------------
    def destroy_node(self):
        """Graceful shutdown."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.get_logger().info(" RadarProcessor stopped.")
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadarProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
