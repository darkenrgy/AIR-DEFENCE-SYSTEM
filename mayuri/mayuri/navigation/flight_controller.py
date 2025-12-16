import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import math
import traceback


try:
    from pymavlink import mavutil
except Exception:
    mavutil = None  


class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        # Parameters
        self.declare_parameter('use_mavlink', False)
        self.declare_parameter('mavlink_connection', 'udpout:127.0.0.1:14550')
        self.declare_parameter('require_confirmation', False)
        self.declare_parameter('telemetry_rate', 1.0)  # Hz
        self.declare_parameter('safety_arm_timeout', 5.0)  # seconds to wait for arm ack
        self.declare_parameter('default_takeoff_alt', 10.0)

        self.use_mavlink = bool(self.get_parameter('use_mavlink').get_parameter_value().bool_value)
        self.mavlink_conn_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        self.require_confirmation = bool(self.get_parameter('require_confirmation').get_parameter_value().bool_value)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').get_parameter_value().double_value or 1.0)
        self.arm_timeout = float(self.get_parameter('safety_arm_timeout').get_parameter_value().double_value or 5.0)
        self.default_takeoff_alt = float(self.get_parameter('default_takeoff_alt').get_parameter_value().double_value or 10.0)

        # ROS topics
        self.cmd_sub = self.create_subscription(String, '/mayuri/flight/cmd', self.on_cmd, 10)
        self.status_pub = self.create_publisher(String, '/mayuri/flight/status', 10)
        self.event_pub = self.create_publisher(String, '/mayuri/flight/event', 10)

        # Internal state
        self.armed = False
        self.mode = 'MANUAL'
        self.last_heartbeat = time.time()
        self.vehicle_position = {'lat': None, 'lon': None, 'alt': None}  # updated from telemetry if available
        self._stop = False
        self._mav = None
        self._mav_lock = threading.Lock()

        # Connect MAVLink if requested
        if self.use_mavlink:
            if mavutil is None:
                self.get_logger().error('pymavlink not installed but use_mavlink=True. Install pymavlink or set use_mavlink=False.')
                raise RuntimeError('pymavlink missing')
            self._connect_mavlink()

        # Telemetry thread: publish status periodically
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        # Optional MAVLink read thread to capture messages
        if self.use_mavlink:
            self.mav_read_thread = threading.Thread(target=self._mav_read_loop, daemon=True)
            self.mav_read_thread.start()

        self.get_logger().info('FlightController initialized.')

    # ---------------------------- MAVLink ---------------------------------
    def _connect_mavlink(self):
        """Establish MAVLink connection (best-effort)."""
        try:
            self.get_logger().info(f'Connecting MAVLink: {self.mavlink_conn_str}')
            self._mav = mavutil.mavlink_connection(self.mavlink_conn_str)
            # Wait for heartbeat
            self.get_logger().info('Waiting for MAVLink heartbeat...')
            self._mav.wait_heartbeat(timeout=10)
            self.get_logger().info('MAVLink heartbeat received.')
        except Exception as e:
            self.get_logger().error(f'Failed MAVLink connect: {e}')
            self._mav = None
            raise

    def _mav_read_loop(self):
        """Continuously read MAVLink messages to update internal state."""
        while not self._stop and rclpy.ok():
            try:
                with self._mav_lock:
                    msg = self._mav.recv_match(blocking=False)
                if msg:
                    mtype = msg.get_type()
                    if mtype == 'HEARTBEAT':
                        self.last_heartbeat = time.time()
                        # update mode/armed if available
                        try:
                            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        except Exception:
                            pass
                    elif mtype == 'GLOBAL_POSITION_INT':
                        # lat/lon are int32 in 1e7; alt relative_alt in mm
                        try:
                            lat = getattr(msg, 'lat', None)
                            lon = getattr(msg, 'lon', None)
                            alt = getattr(msg, 'relative_alt', None)
                            if lat is not None and lon is not None:
                                self.vehicle_position['lat'] = lat / 1e7
                                self.vehicle_position['lon'] = lon / 1e7
                            if alt is not None:
                                self.vehicle_position['alt'] = alt / 1000.0
                        except Exception:
                            pass
                # small sleep to avoid busy loop
                time.sleep(0.02)
            except Exception:
                self.get_logger().warning('Exception in MAV read loop:\n' + traceback.format_exc())
                time.sleep(0.5)

    # ---------------------------- Commands --------------------------------
    def on_cmd(self, msg: String):
        """Receive JSON command on /mayuri/flight/cmd."""
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Invalid JSON command.')
            return

        cmd = payload.get('cmd', '').lower()
        confirm = payload.get('confirm', False)

        if self.require_confirmation and not confirm:
            self.get_logger().warning('Command requires explicit confirm flag.')
            return

        if cmd == 'arm':
            self.arm()
        elif cmd == 'disarm':
            self.disarm()
        elif cmd == 'takeoff':
            alt = float(payload.get('alt', self.default_takeoff_alt))
            self.takeoff(alt)
        elif cmd == 'land':
            self.land()
        elif cmd == 'goto':
            lat = payload.get('lat')
            lon = payload.get('lon')
            alt = payload.get('alt', self.default_takeoff_alt)
            if lat is None or lon is None:
                self.get_logger().warning('goto requires lat and lon.')
            else:
                self.goto(float(lat), float(lon), float(alt))
        elif cmd == 'velocity':
            vx = float(payload.get('vx', 0.0))
            vy = float(payload.get('vy', 0.0))
            vz = float(payload.get('vz', 0.0))
            self.set_velocity(vx, vy, vz)
        elif cmd == 'set_mode':
            mode = payload.get('mode')
            if mode:
                self.set_mode(mode)
        else:
            self.get_logger().warning(f'Unknown flight command: {cmd}')

    # ---------------------------- Primitive Actions -----------------------
    def arm(self):
        """Arm the vehicle (best-effort)."""
        if self.use_mavlink and self._mav:
            self.get_logger().info('Sending ARM command via MAVLink.')
            try:
                with self._mav_lock:
                    # MAV_CMD_COMPONENT_ARM_DISARM param1=1 arm
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1, 0, 0, 0, 0, 0, 0
                    )
                # wait shortly for heartbeat/arm update
                t0 = time.time()
                while time.time() - t0 < self.arm_timeout:
                    if self.armed:
                        break
                    time.sleep(0.2)
                self._publish_event('ARM_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Arm command failed: {e}')
        else:
            # Simulation fallback
            self.armed = True
            self.get_logger().info('Simulated arm -> armed=True')
            self._publish_event('ARM_SIMULATED')

    def disarm(self):
        """Disarm the vehicle (best-effort)."""
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        0, 0, 0, 0, 0, 0, 0
                    )
                self._publish_event('DISARM_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Disarm failed: {e}')
        else:
            self.armed = False
            self.get_logger().info('Simulated disarm -> armed=False')
            self._publish_event('DISARM_SIMULATED')

    def takeoff(self, altitude_m: float):
        """Request takeoff to target altitude (meters)."""
        if not self.armed:
            self.get_logger().warning('Cannot takeoff: vehicle not armed.')
            self._publish_event('TAKEOFF_DENIED_NOT_ARMED')
            return

        self.get_logger().info(f'Requesting takeoff to {altitude_m} m.')
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    # MAV_CMD_NAV_TAKEOFF param7=altitude
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        0,
                        0, 0, 0, 0, 0, 0, float(altitude_m)
                    )
                self._publish_event('TAKEOFF_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Takeoff failed: {e}')
        else:
            # Simulated takeoff: set altitude immediately (simple)
            self.vehicle_position['alt'] = altitude_m
            self._publish_event('TAKEOFF_SIMULATED')
            self.get_logger().info(f'Simulated takeoff -> alt={altitude_m}')

    def land(self):
        """Request landing (best-effort)."""
        self.get_logger().info('Requesting landing.')
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0,
                        0, 0, 0, 0, 0, 0, 0
                    )
                self._publish_event('LAND_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Land failed: {e}')
        else:
            # Simulated land: set altitude to zero
            self.vehicle_position['alt'] = 0.0
            self._publish_event('LAND_SIMULATED')
            self.get_logger().info('Simulated land -> alt=0.0')

    def goto(self, lat: float, lon: float, alt: float):
        """Send a waypoint goto command (best-effort)."""
        self.get_logger().info(f'Goto -> lat:{lat} lon:{lon} alt:{alt}')
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    # Use MAV_CMD_NAV_WAYPOINT (param5=lat, param6=lon, param7=alt)
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0,
                        0, 0, 0, 0,
                        float(lat), float(lon), float(alt)
                    )
                self._publish_event('GOTO_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Goto failed: {e}')
        else:
            # Simulated: set vehicle_position
            self.vehicle_position['lat'] = lat
            self.vehicle_position['lon'] = lon
            self.vehicle_position['alt'] = alt
            self._publish_event('GOTO_SIMULATED')
            self.get_logger().info('Simulated goto -> position updated')

    def set_velocity(self, vx: float, vy: float, vz: float, frame='LOCAL_NED'):
        """
        Send velocity setpoint. vx,vy,vz in m/s.
        frame: 'LOCAL_NED' or 'GLOBAL'
        """
        self.get_logger().info(f'Set velocity -> vx:{vx} vy:{vy} vz:{vz} frame:{frame}')
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    # Use SET_POSITION_TARGET_LOCAL_NED with type_mask to indicate velocity only
                    type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
                    self._mav.mav.set_position_target_local_ned_send(
                        int(time.time() * 1e3),  # time_boot_ms
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        type_mask,
                        0, 0, 0,  # x,y,z position ignored
                        float(vx), float(vy), float(vz),
                        0, 0, 0,  # acceleration
                        0, 0  # yaw, yaw_rate
                    )
                self._publish_event('VELOCITY_COMMAND_SENT')
            except Exception as e:
                self.get_logger().error(f'Velocity command failed: {e}')
        else:
            # Simulated: update a simple position estimate
            try:
                dt = 1.0 / max(1.0, self.telemetry_rate)
                if self.vehicle_position['lat'] is not None and self.vehicle_position['lon'] is not None:
                    # Very rough local update: assume 1 deg lat ~ 111km, lon scaled by cos(lat)
                    lat = self.vehicle_position['lat'] + (vx * dt) / 111000.0
                    lon = self.vehicle_position['lon'] + (vy * dt) / (111000.0 * max(0.001, math.cos(math.radians(self.vehicle_position['lat']))))
                    alt = (self.vehicle_position['alt'] or 0.0) + vz * dt
                    self.vehicle_position['lat'] = lat
                    self.vehicle_position['lon'] = lon
                    self.vehicle_position['alt'] = alt
                self._publish_event('VELOCITY_SIMULATED')
            except Exception as e:
                self.get_logger().warning(f'Velocity simulate error: {e}')

    def set_mode(self, mode_str: str):
        """Set flight mode (best-effort)."""
        self.get_logger().info(f'Requesting mode change: {mode_str}')
        self.mode = mode_str.upper()
        if self.use_mavlink and self._mav:
            try:
                with self._mav_lock:
                    # Best-effort: request using MAV_CMD_DO_SET_MODE (firmware-dependent)
                    self._mav.mav.command_long_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,
                        1, 0, 0, 0, 0, 0, 0
                    )
                self._publish_event('SET_MODE_COMMAND_SENT')
            except Exception as e:
                self.get_logger().warning(f'Set mode attempt failed: {e}')
        else:
            self._publish_event('SET_MODE_SIMULATED')

    # ---------------------------- Utilities -------------------------------
    def _publish_status(self):
        """Publish flight controller status."""
        st = {
            'timestamp': time.time(),
            'armed': self.armed,
            'mode': self.mode,
            'position': self.vehicle_position,
            'last_heartbeat': self.last_heartbeat
        }
        m = String()
        m.data = json.dumps(st)
        self.status_pub.publish(m)

    def _publish_event(self, event_name: str, details: dict = None):
        ev = {
            'timestamp': time.time(),
            'event': event_name,
            'details': details or {}
        }
        m = String()
        m.data = json.dumps(ev)
        self.event_pub.publish(m)
        self.get_logger().info(f'Event: {event_name}')

    def _telemetry_loop(self):
        """Publish status periodically."""
        rate = 1.0 / max(0.01, self.telemetry_rate)
        while not self._stop and rclpy.ok():
            try:
                self._publish_status()
            except Exception as e:
                self.get_logger().warning('Status publish error: ' + str(e))
            time.sleep(rate)

    def destroy_node(self):
        self.get_logger().info('Shutting down FlightController...')
        self._stop = True
        try:
            if self.telemetry_thread.is_alive():
                self.telemetry_thread.join(timeout=1.0)
        except Exception:
            pass
        if self.use_mavlink:
            try:
                if self.mav_read_thread.is_alive():
                    self.mav_read_thread.join(timeout=1.0)
            except Exception:
                pass
            try:
                with self._mav_lock:
                    if self._mav:
                        self._mav.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
