import os
import yaml
import time
import logging
from datetime import datetime
from threading import Thread
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    # ROS2 not available, fallback to simulation mode
    rclpy = None
    Node = object
    String = None

class MissionPlanner(Node if rclpy else object):
    def __init__(self, mission_file: str):
        if rclpy:
            super().__init__("mission_planner")

        self.mission_file = mission_file
        self.mission_data = self._load_mission()
        self.active = False
        self.start_time = None

        # ROS2 Publishers
        if rclpy:
            self.status_pub = self.create_publisher(String, "/mayuri/mission/status", 10)
            self.alert_pub = self.create_publisher(String, "/mayuri/alerts/mission", 10)

        # Logging
        log_dir = Path("logs/missions")
        log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_dir / f"{self.mission_data['mission']['name']}_run.log",
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        logging.info(f"Initialized MissionPlanner for {self.mission_data['mission']['name']}")

    
    def _load_mission(self):
        if not os.path.exists(self.mission_file):
            raise FileNotFoundError(f"Mission file not found: {self.mission_file}")

        with open(self.mission_file, "r") as f:
            mission_data = yaml.safe_load(f)

        logging.info(f"Loaded mission: {mission_data['mission']['name']}")
        return mission_data

    
    def start(self):
        self.active = True
        self.start_time = datetime.now()
        mission_name = self.mission_data["mission"]["name"]

        logging.info(f"Starting mission: {mission_name}")
        self._publish_status(f"Mission '{mission_name}' started.")
        print(f"Mission '{mission_name}' started.")

        # Run mission loop in background
        thread = Thread(target=self._execute_mission, daemon=True)
        thread.start()

    def _execute_mission(self):
        mission = self.mission_data["mission"]
        waypoints = self.mission_data.get("waypoints", [])
        duration = mission.get("max_duration_minutes", 30)
        loop_route = mission.get("loop_route", False)

        # Simulate flight between waypoints
        for idx, wp in enumerate(waypoints):
            if not self.active:
                break

            msg = f"Moving to waypoint {idx+1}/{len(waypoints)}: {wp}"
            self._publish_status(msg)
            logging.info(msg)
            print(f" {msg}")

            # Simulate AI & sensor operations
            self._simulate_ai_scan()

            time.sleep(2)  # Simulated travel time per waypoint

        # Loop route if enabled
        if loop_route and self.active:
            logging.info("Looping route...")
            self._execute_mission()

        # Mission complete
        self.complete()

    
    def _simulate_ai_scan(self):
        if "surveillance" not in self.mission_data:
            return

        ai_conf = self.mission_data["surveillance"].get("ai_detection", {})
        if ai_conf.get("enabled", False):
            conf = ai_conf.get("detection_confidence_threshold", 0.5)
            self._publish_status(f"AI scanning... confidence threshold = {conf}")
            logging.info(f"AI scanning with confidence {conf}")
            time.sleep(1)




    def stop(self, reason="Manual stop"):
        self.active = False
        self._publish_status(f"Mission stopped: {reason}")
        logging.warning(f"Mission stopped: {reason}")
        print(f" Mission stopped: {reason}")

    def complete(self):
        self.active = False
        mission_name = self.mission_data["mission"]["name"]
        duration = (datetime.now() - self.start_time).seconds / 60.0

        msg = f"Mission '{mission_name}' completed in {duration:.2f} min."
        self._publish_status(msg)
        logging.info(msg)
        print(f"🏁 {msg}")



    def _publish_status(self, message):
        if rclpy and String:
            self.status_pub.publish(String(data=message))
        logging.info(message)





    @staticmethod
    def run(mission_file: str):
        if rclpy:
            rclpy.init()
            node = MissionPlanner(mission_file)
            node.start()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.stop("User interrupted")
            finally:
                node.destroy_node()
                rclpy.shutdown()
        else:
            planner = MissionPlanner(mission_file)
            planner.start()
            while planner.active:
                time.sleep(1)
            planner.complete()






if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Mission Planner")
    parser.add_argument(
        "--mission",
        type=str,
        default="config/missions/patrol_mission.yaml",
        help="Path to mission YAML file",
    )
    args = parser.parse_args()

    MissionPlanner.run(args.mission)
