#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import random  # Used for simulating telemetry in test mode


class AutopilotManager(Node):
    def __init__(self):
        super().__init__("autopilot_manager")

        # --- Parameters ---
        self.declare_parameter("test_mode", True)  # If true, simulate autopilot data
        self.declare_parameter("status_publish_rate", 1.0)  # Hz
        self.declare_parameter("heartbeat_timeout", 5.0)  # seconds

        self.test_mode = self.get_parameter("test_mode").value
        self.status_rate = float(self.get_parameter("status_publish_rate").value)
        self.heartbeat_timeout = float(self.get_parameter("heartbeat_timeout").value)

        # --- Publishers ---
        self.status_pub = self.create_publisher(String, "/mayuri/autopilot/status", 10)
        self.event_pub = self.create_publisher(String, "/mayuri/autopilot/event", 10)

        # --- Subscribers ---
        self.command_sub = self.create_subscription(String, "/mayuri/autopilot/cmd", self.handle_command, 10)
        self.telemetry_sub = self.create_subscription(String, "/mayuri/telemetry", self.update_telemetry, 10)

        # --- State ---
        self.last_heartbeat = time.time()
        self.current_mode = "IDLE"
        self.status = {
            "armed": False,
            "mode": self.current_mode,
            "battery": 100.0,
            "altitude": 0.0,
            "gps_lock": True,
            "link_active": True,
            "failsafe": False,
        }

        # --- Threads ---
        self.running = True
        self.status_thread = threading.Thread(target=self.status_loop, daemon=True)
        self.status_thread.start()

        self.health_thread = threading.Thread(target=self.health_monitor_loop, daemon=True)
        self.health_thread.start()

        self.get_logger().info("✓ AutopilotManager initialized and running.")

    # ----------------------------------------------------------------------
    def handle_command(self, msg: String):
        """Handle incoming JSON commands."""
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid command JSON: {e}")
            return

        action = cmd.get("action", "").lower()
        self.get_logger().info(f"Received autopilot command: {action}")

        if action == "arm":
            self.arm()
        elif action == "disarm":
            self.disarm()
        elif action == "takeoff":
            alt = cmd.get("alt", 10)
            self.takeoff(alt)
        elif action == "land":
            self.land()
        elif action == "rtl":
            self.return_to_launch()
        elif action == "hold":
            self.hold_position()
        elif action == "set_mode":
            mode = cmd.get("mode", "AUTO")
            self.set_mode(mode)
        else:
            self.get_logger().warning(f"Unknown autopilot command: {action}")

    # ----------------------------------------------------------------------
    def arm(self):
        self.status["armed"] = True
        self.publish_event("ARMED")
        self.get_logger().info("✓ Vehicle armed.")

    def disarm(self):
        self.status["armed"] = False
        self.publish_event("DISARMED")
        self.get_logger().info("Vehicle disarmed.")

    def takeoff(self, alt):
        if not self.status["armed"]:
            self.publish_event("DENIED_TAKEOFF_NOT_ARMED")
            self.get_logger().warning("Takeoff denied: vehicle not armed.")
            return
        self.status["mode"] = "TAKEOFF"
        self.status["altitude"] = alt
        self.publish_event(f"TAKEOFF_INITIATED to {alt}m")
        self.get_logger().info(f" Taking off to {alt} meters (simulation).")

    def land(self):
        self.status["mode"] = "LAND"
        self.publish_event("LANDING_INITIATED")
        self.get_logger().info(" Landing sequence started.")

    def return_to_launch(self):
        self.status["mode"] = "RTL"
        self.publish_event("RETURN_TO_LAUNCH")
        self.get_logger().info(" Return-to-launch activated.")

    def hold_position(self):
        self.status["mode"] = "HOLD"
        self.publish_event("HOLD_POSITION")
        self.get_logger().info("Holding position.")

    def set_mode(self, mode):
        self.status["mode"] = mode.upper()
        self.publish_event(f"MODE_CHANGED_{mode.upper()}")
        self.get_logger().info(f" Mode changed to {mode.upper()}")

    # ----------------------------------------------------------------------
    def update_telemetry(self, msg: String):
        """Receive and process telemetry data."""
        try:
            data = json.loads(msg.data)
            telemetry = data.get("telemetry", {})
            battery = telemetry.get("battery", {}).get("battery_remaining")
            position = telemetry.get("position", {})

            if battery is not None:
                self.status["battery"] = float(battery)
            if position:
                self.status["altitude"] = position.get("alt", self.status["altitude"])

            self.last_heartbeat = time.time()

        except Exception as e:
            self.get_logger().warning(f"Telemetry parse error: {e}")

    # ----------------------------------------------------------------------
    def publish_event(self, message: str):
        """Publish an event for system monitoring."""
        event = {"timestamp": time.time(), "event": message}
        msg = String()
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)

    # ----------------------------------------------------------------------
    def status_loop(self):
        """Periodically publish autopilot status."""
        rate = 1.0 / max(0.01, self.status_rate)
        while self.running and rclpy.ok():
            try:
                if self.test_mode:
                    # Simulate slight battery drop and random altitude drift
                    self.status["battery"] = max(0.0, self.status["battery"] - 0.05)
                    if self.status["mode"] == "TAKEOFF":
                        self.status["altitude"] += random.uniform(0.1, 0.5)
                status_msg = String()
                status_msg.data = json.dumps({
                    "timestamp": time.time(),
                    "status": self.status,
                })
                self.status_pub.publish(status_msg)
            except Exception as e:
                self.get_logger().warning(f"Status loop error: {e}")
            time.sleep(rate)

    # ----------------------------------------------------------------------
    def health_monitor_loop(self):
        """Monitors link status, heartbeat, and triggers failsafes."""
        while self.running and rclpy.ok():
            now = time.time()
            if now - self.last_heartbeat > self.heartbeat_timeout:
                if self.status["link_active"]:
                    self.status["link_active"] = False
                    self.publish_event("LINK_LOST")
                    self.get_logger().warning(" Telemetry link lost.")
                    self.trigger_failsafe("link_loss")
            else:
                self.status["link_active"] = True
            time.sleep(1.0)

    # ----------------------------------------------------------------------
    def trigger_failsafe(self, reason: str):
        """Invoke a failsafe mode depending on reason."""
        self.status["failsafe"] = True
        if reason == "link_loss":
            self.return_to_launch()
        elif reason == "low_battery":
            self.land()
        else:
            self.hold_position()
        self.publish_event(f"FAILSAFE_TRIGGERED_{reason.upper()}")

    # ----------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if self.status_thread.is_alive():
            self.status_thread.join(timeout=1.0)
        if self.health_thread.is_alive():
            self.health_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutopilotManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
