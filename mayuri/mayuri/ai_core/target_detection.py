import cv2
import json
import torch
import time
import numpy as np
import logging
from pathlib import Path
from datetime import datetime

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None

class TargetDetector(Node if rclpy else object):
    def __init__(self, model_path="ai_core/models/yolo_defense.onnx", confidence_threshold=0.45):
        if rclpy:
            super().__init__("target_detector")

        self.model_path = model_path

        self.confidence_threshold = confidence_threshold

        # Logging setup
        log_dir = Path("logs/ai")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "target_detection.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher setup
        if rclpy:
            self.detection_pub = self.create_publisher(String, "/mayuri/ai/target_detections", 10)

        logging.info(f"TargetDetector initialized with model {model_path}")

        # Load YOLO/ONNX model
        self.model = self._load_model()

    def _load_model(self):
        if YOLO:
            try:
                model = YOLO(self.model_path)
                logging.info("Loaded YOLO model successfully.")
                return model
            except Exception as e:
                logging.error(f"Failed to load YOLO model: {e}")
                raise e
        else:
            raise ImportError("Ultralytics YOLO not found. Install with `pip install ultralytics`.")

    def detect(self, frame):
        try:
            results = self.model.predict(frame, conf=self.confidence_threshold, verbose=False)

            detections = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = self.model.names[cls]

                    detections.append({
                        "label": label,
                        "confidence": round(conf, 3),
                        "bbox": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                    })

            if detections:
                self._publish_detections(detections)
                logging.info(f"Detected {len(detections)} objects.")
            return detections

        except Exception as e:
            logging.error(f"Detection error: {e}")
            return []

    def _publish_detections(self, detections):
        if rclpy and String:
            msg = String()
            msg.data = json.dumps({
                "timestamp": datetime.utcnow().isoformat(),
                "detections": detections
            })
            self.detection_pub.publish(msg)

    def run_stream(self, source=0):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(" Unable to access video source.")
            return

        print("Starting live detection... Press 'q' to quit.")
        fps_time = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            detections = self.detect(frame)

            # Draw bounding boxes
            for det in detections:
                (x, y, w, h) = det["bbox"]
                label = det["label"]
                conf = det["confidence"]

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf*100:.1f}%", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # FPS counter
            fps = 1.0 / (time.time() - fps_time)
            fps_time = time.time()
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("MAYURI - Target Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        print(" Stopping Target Detector...")
        if rclpy:
            self.destroy_node()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Target Detection Engine")
    parser.add_argument("--model", type=str, default="ai_core/models/yolo_defense.onnx", help="Path to YOLO model")
    parser.add_argument("--source", type=str, default="0", help="Camera index or video path")
    parser.add_argument("--conf", type=float, default=0.45, help="Detection confidence threshold")
    args = parser.parse_args()

    detector = TargetDetector(model_path=args.model, confidence_threshold=args.conf)
    detector.run_stream(int(args.source) if args.source.isdigit() else args.source)
