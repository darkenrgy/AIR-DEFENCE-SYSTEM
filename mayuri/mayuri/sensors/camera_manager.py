import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import threading
import time
from cv_bridge import CvBridge


class CameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager')

        # Parameters
        self.declare_parameter('rgb_source', 0)  # Default USB camera
        self.declare_parameter('thermal_source', '')
        self.declare_parameter('depth_source', '')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('simulation_mode', False)

        self.rgb_source = self.get_parameter('rgb_source').value
        self.thermal_source = self.get_parameter('thermal_source').value
        self.depth_source = self.get_parameter('depth_source').value
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)

        # ROS2 publishers
        self.rgb_pub = self.create_publisher(Image, '/mayuri/camera/rgb', 10)
        self.thermal_pub = self.create_publisher(Image, '/mayuri/camera/thermal', 10)
        self.depth_pub = self.create_publisher(Image, '/mayuri/camera/depth', 10)
        self.status_pub = self.create_publisher(String, '/mayuri/camera/status', 10)

        self.bridge = CvBridge()
        self.running = True

        # Threads for each camera type
        self.rgb_thread = threading.Thread(target=self.capture_rgb, daemon=True)
        self.rgb_thread.start()

        if self.thermal_source:
            self.thermal_thread = threading.Thread(target=self.capture_thermal, daemon=True)
            self.thermal_thread.start()

        if self.depth_source:
            self.depth_thread = threading.Thread(target=self.capture_depth, daemon=True)
            self.depth_thread.start()

        self.get_logger().info(" CameraManager initialized and streaming started.")

    # ----------------------------------------------------------------------
    def capture_rgb(self):
        """Capture and publish RGB camera frames."""
        cap = self._open_stream(self.rgb_source, "RGB")
        if not cap:
            return

        frame_interval = 1.0 / self.frame_rate
        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self._handle_camera_failure("RGB")
                break

            if self.publish_raw:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.rgb_pub.publish(msg)

            self._publish_status("RGB_ACTIVE", frame.shape)
            time.sleep(frame_interval)

        cap.release()

    def capture_thermal(self):
        """Capture thermal camera frames."""
        cap = self._open_stream(self.thermal_source, "THERMAL")
        if not cap:
            return

        frame_interval = 1.0 / self.frame_rate
        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self._handle_camera_failure("THERMAL")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            self.thermal_pub.publish(msg)

            self._publish_status("THERMAL_ACTIVE", gray.shape)
            time.sleep(frame_interval)

        cap.release()

    def capture_depth(self):
        """Capture depth frames."""
        cap = self._open_stream(self.depth_source, "DEPTH")
        if not cap:
            return

        frame_interval = 1.0 / self.frame_rate
        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self._handle_camera_failure("DEPTH")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono16')
            self.depth_pub.publish(msg)

            self._publish_status("DEPTH_ACTIVE", gray.shape)
            time.sleep(frame_interval)

        cap.release()

    # ----------------------------------------------------------------------
    def _open_stream(self, source, cam_type):
        """Helper: open a camera or video source."""
        try:
            if self.simulation_mode and isinstance(source, str):
                self.get_logger().info(f" Simulated {cam_type} source: {source}")
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                raise RuntimeError(f"{cam_type} camera source not available: {source}")
            self.get_logger().info(f"✓ {cam_type} camera started ({source})")
            return cap
        except Exception as e:
            self.get_logger().error(f"{cam_type} stream open failed: {e}")
            self._publish_status(f"{cam_type}_FAILED", None)
            return None

    def _handle_camera_failure(self, cam_type):
        """Log and publish failure."""
        self.get_logger().error(f"{cam_type} camera feed lost.")
        self._publish_status(f"{cam_type}_ERROR", None)

    def _publish_status(self, status, shape):
        """Publish camera health status."""
        info = {
            "timestamp": time.time(),
            "status": status,
            "shape": shape if shape else None
        }
        msg = String()
        msg.data = json.dumps(info)
        self.status_pub.publish(msg)

    # ----------------------------------------------------------------------
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        self.get_logger().info("Stopping all camera streams...")
        time.sleep(0.5)
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
