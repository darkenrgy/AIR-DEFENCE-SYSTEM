import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading


class FailsafeManager(Node):
    def __init__(self):
        super().__init__("failsafe_manager")

        # --- Parameters ---
        self.declare_parameter("battery_min_threshold", 15.0)  # %
        self.declare_parameter("gps_required", True)
        self.declare_parameter("link_timeout", 5.0)  # seconds
        self.declare_parameter("sensor_timeout", 4.0)
        self.declare_parameter("check_rate", 1.0)  # Hz

        self.battery_min = float(self.get_parameter("battery_min_threshold").value)
        self.gps_required = bool(self.get_parameter("gps_required").value)
        self.link_timeout = float(self.get_parameter("link_timeout").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.check_rate = float(self.get_parameter("check_rate").value)

        # --- State variables ---
        self.last_telemetry = time.time()
        self.last_sensor = time.time()
        self.last_gps_fix = time.time()
        self.last_battery = 100.0
        self.link_active = True
        self.failsafe_triggered = False
        self.current_mode = "NORMAL"

        # --- Publishers ---
        self.failsafe_pub = self.create_publisher(String, "/mayuri/failsafe/event", 10)
        self.command_pub = self.create_publisher(String, "/mayuri/autopilot/cmd", 10)

        # --- Subscribers ---
        self.telemetry_sub = self.create_subscription(String, "/mayuri/telemetry", self.on_telemetry, 10)
        self.sensor_sub = self.create_subscription(String, "/mayuri/sensor/status", self.on_sensor_update, 10)
        self.gps_sub = self.create_subscription(String, "/mayuri/gps/status", self.on_gps_update, 10)

        # --- Threads ---
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info("✓ FailsafeManager initialized and running.")

    # ----------------------------------------------------------------------
    def on_telemetry(self, msg: String):
        """Update telemetry info."""
        try:
            data = json.loads(msg.data)
            telemetry = data.get("telemetry", {})
            battery = telemetry.get("battery", {}).get("battery_remaining")
            if battery is not None:
                self.last_battery = float(battery)
            self.last_telemetry = time.time()
        except Exception as e:
            self.get_logger().warning(f"Telemetry parse error: {e}")

    # ----------------------------------------------------------------------
    def on_sensor_update(self, msg: String):
        """Update sensor heartbeat."""
        self.last_sensor = time.time()

    def on_gps_update(self, msg: String):
        """Update GPS status."""
        try:
            data = json.loads(msg.data)
            gps_lock = data.get("gps_lock", True)
            if gps_lock:
                self.last_gps_fix = time.time()
        except Exception:
            pass

    # ----------------------------------------------------------------------
    def monitor_loop(self):
        """Main loop checking system health and triggering failsafes."""
        rate = 1.0 / max(0.01, self.check_rate)
        while self.running and rclpy.ok():
            now = time.time()

            # --- Link health ---
            if now - self.last_telemetry > self.link_timeout:
                if self.link_active:
                    self.link_active = False
                    self.trigger_failsafe("LINK_LOSS", "hold")
            else:
                self.link_active = True

            # --- Battery ---
            if self.last_battery <= self.battery_min:
                self.trigger_failsafe("LOW_BATTERY", "land")

            # --- Sensor ---
            if now - self.last_sensor > self.sensor_timeout:
                self.trigger_failsafe("SENSOR_FAILURE", "hold")

            # --- GPS ---
            if self.gps_required and (now - self.last_gps_fix > self.sensor_timeout):
                self.trigger_failsafe("GPS_LOSS", "hold")

            time.sleep(rate)

    # ----------------------------------------------------------------------
    def trigger_failsafe(self, reason: str, action: str):
        """Handle any fail-safe trigger."""
        if self.failsafe_triggered:
            return  # avoid multiple triggers
        self.failsafe_triggered = True
        self.current_mode = "FAILSAFE"

        event = {
            "timestamp": time.time(),
            "reason": reason,
            "action": action,
        }

        msg = String()
        msg.data = json.dumps(event)
        self.failsafe_pub.publish(msg)

        self.get_logger().warning(f"FAILSAFE TRIGGERED: {reason} → {action.upper()}")

        # Notify autopilot_manager for safe action
        cmd = {"action": action}
        cmd_msg = String()
        cmd_msg.data = json.dumps(cmd)
        self.command_pub.publish(cmd_msg)

    # ----------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FailsafeManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
