import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random
import math
from datetime import datetime


class TestFlightStatus(Node):
    def __init__(self):
        super().__init__('test_flight_status')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/mayuri/flight/status', 10)
        self.waypoint_pub = self.create_publisher(String, '/mayuri/mission/waypoints', 10)
        
        # Timer for status updates (5 Hz)
        self.create_timer(0.2, self.publish_status)
        
        # Timer for waypoint updates (once every 30 seconds)
        self.create_timer(30.0, self.publish_waypoints)
        
        # Flight parameters
        self.start_lat = 12.9716  # Bangalore
        self.start_lon = 77.5946
        self.current_lat = self.start_lat
        self.current_lon = self.start_lon
        self.current_alt = 0.0
        self.heading = 0.0
        
        # Flight state
        self.armed = False
        self.battery = 100.0
        self.flight_mode = "STABILIZE"
        self.time_elapsed = 0.0
        
        # Mission simulation
        self.mission_phase = 0  # 0=ground, 1=takeoff, 2=cruise, 3=land
        
        self.get_logger().info(" Test Flight Status Publisher Started")
        self.get_logger().info(f"   Starting position: {self.start_lat:.6f}, {self.start_lon:.6f}")
        self.get_logger().info("   Simulating realistic flight patterns...")
        
        # Publish initial waypoints
        self.publish_waypoints()
    
    
    def publish_status(self):
        """Publish simulated flight status"""
        
        self.time_elapsed += 0.2
        
        # Simulate mission phases
        if self.time_elapsed < 10:
            # Ground phase
            self.mission_phase = 0
            self.armed = True
            self.flight_mode = "STABILIZE"
            self.current_alt = 0.0
        
        elif self.time_elapsed < 20:
            # Takeoff phase
            self.mission_phase = 1
            self.flight_mode = "GUIDED"
            self.current_alt = min(50.0, self.current_alt + 0.5)
        
        elif self.time_elapsed < 120:
            # Cruise phase - circular pattern
            self.mission_phase = 2
            self.flight_mode = "AUTO"
            self.current_alt = 50.0 + 10.0 * math.sin(self.time_elapsed * 0.1)
            
            # Circular flight pattern
            radius = 0.002  # degrees (~220m)
            angular_speed = 0.05  # radians per second
            angle = self.time_elapsed * angular_speed
            
            self.current_lat = self.start_lat + radius * math.cos(angle)
            self.current_lon = self.start_lon + radius * math.sin(angle)
            self.heading = (math.degrees(angle) + 90) % 360
        
        else:
            # Landing phase
            self.mission_phase = 3
            self.flight_mode = "LAND"
            self.current_alt = max(0.0, self.current_alt - 0.3)
            
            # Reset after landing
            if self.current_alt <= 0.1:
                self.time_elapsed = 0.0
                self.battery = 100.0
        
        # Battery drain (faster during flight)
        drain_rate = 0.02 if self.current_alt > 1 else 0.005
        self.battery = max(0.0, self.battery - drain_rate)
        
        # Calculate velocity
        if self.mission_phase == 2:  # Cruising
            vx = 5.0 * math.cos(math.radians(self.heading))
            vy = 5.0 * math.sin(math.radians(self.heading))
            vz = 0.5 * math.sin(self.time_elapsed * 0.1)
        elif self.mission_phase == 1:  # Climbing
            vx, vy = 0.0, 0.0
            vz = 2.5
        elif self.mission_phase == 3:  # Descending
            vx, vy = 0.0, 0.0
            vz = -1.5
        else:  # Ground
            vx, vy, vz = 0.0, 0.0, 0.0
        
        # Add small random variations
        vx += random.uniform(-0.5, 0.5)
        vy += random.uniform(-0.5, 0.5)
        self.heading += random.uniform(-2, 2)
        self.heading = self.heading % 360
        
        # GPS satellites
        gps_sats = random.randint(12, 18)
        
        # Create status message
        status = {
            "position": {
                "lat": self.current_lat,
                "lon": self.current_lon,
                "alt": self.current_alt
            },
            "velocity": {
                "vx": vx,
                "vy": vy,
                "vz": vz
            },
            "heading": self.heading,
            "mode": self.flight_mode,
            "armed": self.armed,
            "battery": {
                "battery_remaining": int(self.battery)
            },
            "gps_satellites": gps_sats,
            "gps_ok": True,
            "timestamp": datetime.now().isoformat()
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
        # Log periodically
        if int(self.time_elapsed) % 10 == 0:
            self.get_logger().info(
                f" Mode={self.flight_mode}, Alt={self.current_alt:.1f}m, "
                f"Battery={self.battery:.0f}%, Heading={self.heading:.0f}°"
            )
    
    
    def publish_waypoints(self):
        """Publish mission waypoints"""
        
        # Create a square pattern around start position
        offset = 0.002  # ~220m
        
        waypoints_data = {
            "waypoints": [
                {"lat": self.start_lat + offset, "lon": self.start_lon, "alt": 50},
                {"lat": self.start_lat + offset, "lon": self.start_lon + offset, "alt": 60},
                {"lat": self.start_lat, "lon": self.start_lon + offset, "alt": 50},
                {"lat": self.start_lat, "lon": self.start_lon, "alt": 40},
            ]
        }
        
        msg = String()
        msg.data = json.dumps(waypoints_data)
        self.waypoint_pub.publish(msg)
        
        self.get_logger().info(f" Published {len(waypoints_data['waypoints'])} waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = TestFlightStatus()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
