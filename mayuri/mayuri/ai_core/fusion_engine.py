import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np


class MayuriClassifier(Node):
    def __init__(self):
        super().__init__('mayuri_classifier')

        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

        # Publishers
        self.label_pub = self.create_publisher(String, "/mayuri/classification", 10)
        self.bbox_pub = self.create_publisher(MarkerArray, "/mayuri/detections/bboxes", 10)
        self.position_pub = self.create_publisher(PointStamped, "/mayuri/detections/positions", 10)


        self.get_logger().info("Mayuri Classifier with BBoxes + Position Estimate running...")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_h, img_w, _ = frame.shape

        results = self.model(frame)[0]

        labels = []
        marker_array = MarkerArray()

        for i, box in enumerate(results.boxes):
            cls_id = int(box.cls)
            label = self.model.names[cls_id]
            conf = float(box.conf)
            labels.append(f"{label} ({conf:.2f})")

            # Bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # 2D object center
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # -------- Publish 2D Position --------
            pos_msg = PointStamped()
            pos_msg.header.stamp = self.get_clock().now().to_msg()
            pos_msg.point.x = float(cx)
            pos_msg.point.y = float(cy)
            pos_msg.point.z = 0.0
            self.position_pub.publish(pos_msg)

            # -------- Publish Bounding Box Marker --------
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mayuri_bbox"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.scale.x = (x2 - x1) / img_w
            marker.scale.y = (y2 - y1) / img_h
            marker.scale.z = 0.01

            marker.pose.position.x = (cx - img_w / 2) / img_w
            marker.pose.position.y = (cy - img_h / 2) / img_h
            marker.pose.position.z = 0.0

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.1
            marker.color.a = 0.7

            marker_array.markers.append(marker)


        # Publish bounding boxes
        if marker_array.markers:
            self.bbox_pub.publish(marker_array)

        # Publish labels
        msg_out = String()
        msg_out.data = ", ".join(labels) if labels else "no-object"
        self.label_pub.publish(msg_out)

        self.get_logger().info(f"Detected: {msg_out.data}")


    # ---------------- MAVROS FOLLOW LOGIC ----------------
    def follow_target(self, cx, cy, w, h):
        """
        Drone follows the detected object.
        cx, cy = pixel center of object
        """
        center_x = w / 2
        center_y = h / 2

        # Error (object deviation from center)
        error_x = (cx - center_x) / w
        error_y = (cy - center_y) / h

        # Threshold for small noise
        if abs(error_x) < 0.05 and abs(error_y) < 0.05:
            self.get_logger().info("Target centered — hover")
            return

        # Example simple control
        # pose = PoseStamped()
        # pose.pose.position.x += -error_y * 0.5  move forward/back
        # pose.pose.position.y += -error_x * 0.5  move left/right
        # pose.pose.position.z = 2.0
        # self.mavros_pub.publish(pose)

        self.get_logger().info(f"Following target | error_x={error_x:.2f} error_y={error_y:.2f}")



def main(args=None):
    rclpy.init(args=args)
    node = MayuriClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()