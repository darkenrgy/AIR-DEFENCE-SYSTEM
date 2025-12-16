import cv2
import numpy as np
import json
import time
import logging
from pathlib import Path
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None

class MotionTracker(Node if rclpy else object):
    def __init__(self, max_lost=10, tracker_type="KCF"):
        if rclpy:
            super().__init__("motion_tracker")
        else:
            super().__init__()

        self.tracker_type = tracker_type
        self.trackers = []
        self.objects = {}
        self.max_lost = max_lost
        self.frame_count = 0

        # Logging setup
        log_dir = Path("logs/ai")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / "motion_tracking.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher
        if rclpy:
            self.track_pub = self.create_publisher(String, "/mayuri/ai/motion_tracking", 10)

        logging.info(f"MotionTracker initialized using {tracker_type}")

    def _create_tracker(self):
        trackers_map = {
            "KCF": cv2.TrackerKCF_create,
            "CSRT": cv2.TrackerCSRT_create,
            "MOSSE": cv2.TrackerMOSSE_create
        }
        tracker_func = trackers_map.get(self.tracker_type, cv2.TrackerKCF_create)
        return tracker_func()

    def initialize(self, frame, detections):
        """
        detections: list of bounding boxes [x, y, w, h]
        """
        self.trackers = []
        for i, det in enumerate(detections):
            tracker = self._create_tracker()
            x, y, w, h = det
            success = tracker.init(frame, (x, y, w, h))
            if success:
                self.trackers.append(tracker)
                self.objects[i] = {"lost": 0}
        logging.info(f"Initialized {len(self.trackers)} trackers")

    def update(self, frame):
        tracked_objects = []

        for i, tracker in enumerate(self.trackers):
            success, bbox = tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                tracked_objects.append({
                    "id": i,
                    "bbox": [x, y, w, h],
                    "center": [x + w // 2, y + h // 2]
                })
                self.objects[i]["lost"] = 0
            else:
                self.objects[i]["lost"] += 1

        # Remove lost trackers
        valid_indices = [i for i in self.objects if self.objects[i]["lost"] < self.max_lost]
        self.trackers = [self.trackers[i] for i in valid_indices if i < len(self.trackers)]

        if tracked_objects:
            self._publish_data(tracked_objects)
            logging.info(f"Tracked {len(tracked_objects)} objects.")
        else:
            logging.warning("No objects tracked!")

        return tracked_objects

    def _publish_data(self, tracked_objects):
        if rclpy and String:
            msg = String()
            msg.data = json.dumps({
                "timestamp": datetime.utcnow().isoformat(),
                "objects": tracked_objects
            })
            self.track_pub.publish(msg)

    def run_stream(self, source=0):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print("Could not open video source.")
            return

        print("Press SPACE to select ROI, ENTER to confirm, Q to quit.")
        ret, frame = cap.read()
        if not ret:
            print("No frames received.")
            return

        # Manually select objects to track
        boxes = []
        while True:
            box = cv2.selectROI("MAYURI - Select Objects", frame, fromCenter=False, showCrosshair=True)
            boxes.append(box)
            print(f"Added ROI: {box}")
            key = cv2.waitKey(0)
            if key == 13:  # Enter
                break

        self.initialize(frame, boxes)
        cv2.destroyAllWindows()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            tracked = self.update(frame)
            for obj in tracked:
                (x, y, w, h) = obj["bbox"]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"ID {obj['id']}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("MAYURI - Motion Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()



    def stop(self):
        print("Stopping Motion Tracker...")
        if rclpy:
            self.destroy_node()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Motion Tracking Module")
    parser.add_argument("--source", type=str, default="0", help="Camera index or video path")
    parser.add_argument("--tracker", type=str, default="KCF", help="Tracker type: KCF, CSRT, MOSSE")
    args = parser.parse_args()

    tracker = MotionTracker(tracker_type=args.tracker)
    tracker.run_stream(int(args.source) if args.source.isdigit() else args.source)
