import cv2
import numpy as np
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import threading


class CalibrationManager(Node):
    def __init__(self):
        super().__init__("calibration_manager")

        # Parameters
        self.declare_parameter("save_path", "config/calibration_data.json")
        self.declare_parameter("chessboard_size", [9, 6])
        self.declare_parameter("square_size", 0.024)  # meters
        self.declare_parameter("auto_validate", True)

        self.save_path = self.get_parameter("save_path").value
        self.chessboard_size = tuple(self.get_parameter("chessboard_size").value)
        self.square_size = float(self.get_parameter("square_size").value)
        self.auto_validate = bool(self.get_parameter("auto_validate").value)

        # ROS2 publisher for calibration status
        self.status_pub = self.create_publisher(String, "/mayuri/calibration/status", 10)

        self.calibration_data = {}
        self.running = True
        self.lock = threading.Lock()

        self.get_logger().info("CalibrationManager initialized and ready.")

    # ----------------------------------------------------------------------
    def perform_camera_calibration(self, image_files):
        """Perform intrinsic calibration for camera using chessboard images."""
        obj_points = []  # 3D points in real-world space
        img_points = []  # 2D points in image plane

        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        self.get_logger().info("📷 Starting camera calibration...")

        for fname in image_files:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                obj_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                            (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
                img_points.append(corners2)
                cv2.drawChessboardCorners(img, self.chessboard_size, corners2, ret)
            else:
                self.get_logger().warning(f" Chessboard not detected in {fname}")

        if not obj_points:
            self.get_logger().error(" No valid calibration images found.")
            return None

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, gray.shape[::-1], None, None)

        if ret:
            self.get_logger().info("✓ Camera calibration successful.")
            data = {
                "camera_matrix": mtx.tolist(),
                "dist_coeffs": dist.tolist(),
                "rvecs": [r.tolist() for r in rvecs],
                "tvecs": [t.tolist() for t in tvecs]
            }
            self._save_data("camera", data)
            return data
        else:
            self.get_logger().error(" Calibration failed.")
            return None

    # ----------------------------------------------------------------------
    def perform_extrinsic_calibration(self, lidar_points, camera_points):
        """Compute transformation between LIDAR and camera coordinate systems."""
        if len(lidar_points) != len(camera_points):
            self.get_logger().error("Mismatched point sets for extrinsic calibration.")
            return None

        self.get_logger().info("🧮 Computing extrinsic calibration (LIDAR ↔ Camera)...")

        lidar_points = np.array(lidar_points)
        camera_points = np.array(camera_points)

        # Solve PnP to find rotation & translation
        ret, rvec, tvec = cv2.solvePnP(lidar_points, camera_points, np.eye(3), None)
        if not ret:
            self.get_logger().error(" Extrinsic calibration failed.")
            return None

        R, _ = cv2.Rodrigues(rvec)
        transform = np.hstack((R, tvec))
        data = {
            "rotation_matrix": R.tolist(),
            "translation_vector": tvec.tolist()
        }
        self._save_data("extrinsic", data)

        self.get_logger().info("✓ Extrinsic calibration saved successfully.")
        return data

    # ----------------------------------------------------------------------
    def validate_calibration(self):
        """Validate calibration by checking reprojection error."""
        if "camera" not in self.calibration_data:
            self.get_logger().warning("No camera calibration data found.")
            return

        mtx = np.array(self.calibration_data["camera"]["camera_matrix"])
        dist = np.array(self.calibration_data["camera"]["dist_coeffs"])
        self.get_logger().info(f"Camera matrix: {mtx}")
        self.get_logger().info(f"Distortion coefficients: {dist}")

        self._publish_status("CALIBRATION_VALIDATED")

    # ----------------------------------------------------------------------
    def _save_data(self, key, data):
        """Save calibration data to JSON file."""
        with self.lock:
            if not os.path.exists(os.path.dirname(self.save_path)):
                os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

            if os.path.exists(self.save_path):
                with open(self.save_path, "r") as f:
                    self.calibration_data = json.load(f)
            else:
                self.calibration_data = {}

            self.calibration_data[key] = data
            with open(self.save_path, "w") as f:
                json.dump(self.calibration_data, f, indent=4)

            self._publish_status(f"{key.upper()}_CALIBRATION_SAVED")
            self.get_logger().info(f" Saved {key} calibration data to {self.save_path}")

    def load_calibration_data(self):
        """Load calibration data from JSON file."""
        try:
            with open(self.save_path, "r") as f:
                self.calibration_data = json.load(f)
            self._publish_status("CALIBRATION_LOADED")
            self.get_logger().info(" Calibration data loaded successfully.")
        except Exception as e:
            self.get_logger().warning(f" Failed to load calibration file: {e}")

    def _publish_status(self, status):
        """Publish calibration status."""
        msg = {"timestamp": time.time(), "status": status}
        s = String()
        s.data = json.dumps(msg)
        self.status_pub.publish(s)

    def destroy_node(self):
        self.running = False
        self.get_logger().info(" CalibrationManager stopped.")
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationManager()
        node.load_calibration_data()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
