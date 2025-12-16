import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from pymavlink import mavutil

class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')

        # Parameters
        self.declare_parameter('mavlink_connection', 'udpout:127.0.0.1:14550')  # use SITL default
        self.declare_parameter('require_confirmation', False)  # if True, requires "confirm": true in command
        self.declare_parameter('telemetry_pub_rate', 1.0)  # Hz

        conn_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        self.require_confirmation = self.get_parameter('require_confirmation').get_parameter_value().bool_value
        self.telemetry_rate = float(self.get_parameter('telemetry_pub_rate').get_parameter_value().double_value or 1.0)

        # ROS publishers / subscribers
        self.cmd_sub = self.create_subscription(String, '/mayuri/command', self.on_command, 10)
        self.telemetry_pub = self.create_publisher(String, '/mayuri/telemetry', 10)

        # Connect to MAVLink
        self.get_logger().info(f'Connecting to MAVLink: {conn_str}')
        try:
            # mavutil will create the connection and (for UDP) send heartbeats
            self.mav = mavutil.mavlink_connection(conn_str)
            # Wait for a heartbeat so we know the autopilot is alive
            self.get_logger().info('Waiting for MAVLink heartbeat (this may take a few seconds)...')
            self.mav.wait_heartbeat(timeout=10)
            self.get_logger().info('Heartbeat received from system: target_system=%s target_component=%s' %
                                   (self.mav.target_system, self.mav.target_component))
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MAVLink: {e}')
            raise

        # Start telemetry thread
        self._stop = False
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        self.get_logger().info('CommandBridge initialized and running.')

    # --- Command handling ---
    def on_command(self, msg: String):
        """
        Expects msg.data to be a JSON string, for example:
        {"cmd":"arm", "confirm": true}
        {"cmd":"takeoff", "alt": 10, "confirm": true}
        {"cmd":"land", "confirm": true}
        {"cmd":"goto", "lat": 12.34, "lon": 77.56, "alt": 20, "confirm": true}
        {"cmd":"set_mode", "mode":"GUIDED", "confirm": true}
        """
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning(f'Invalid command JSON: {e}')
            return

        cmd = payload.get('cmd', '').lower()
        confirm = payload.get('confirm', False)

        if self.require_confirmation and not confirm:
            self.get_logger().warning('Command requires explicit confirm: set "confirm": true in payload.')
            return

        # Map commands
        if cmd == 'arm':
            self._arm()
        elif cmd == 'disarm':
            self._disarm()
        elif cmd == 'takeoff':
            alt = float(payload.get('alt', 10.0))
            self._takeoff(alt)
        elif cmd == 'land':
            self._land()
        elif cmd == 'goto':
            lat = float(payload.get('lat', 0.0))
            lon = float(payload.get('lon', 0.0))
            alt = float(payload.get('alt', 10.0))
            self._goto(lat, lon, alt)
        elif cmd == 'set_mode':
            mode = payload.get('mode', '')
            if mode:
                self._set_mode(mode)
            else:
                self.get_logger().warning('set_mode requires "mode" parameter.')
        else:
            self.get_logger().warning(f'Unknown command: {cmd}')

    # --- MAVLink command implementations ---

    def _send_command_long(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """Utility to send COMMAND_LONG and wait for ACK (best-effort)."""
        try:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                command,
                0,  # confirmation
                float(param1),
                float(param2),
                float(param3),
                float(param4),
                float(param5),
                float(param6),
                float(param7)
            )
            # non-blocking; try to read an ACK for short time
            ack = None
            t0 = time.time()
            while time.time() - t0 < 2.0:
                msg = self.mav.recv_match(type='COMMAND_ACK', blocking=False)
                if msg:
                    if int(msg.command) == int(command):
                        ack = msg
                        break
                time.sleep(0.05)
            if ack:
                self.get_logger().info(f'CMD {command} ACK: result={ack.result}')
            else:
                self.get_logger().info(f'CMD {command} sent (no ACK observed).')
        except Exception as e:
            self.get_logger().error(f'Failed to send COMMAND_LONG {command}: {e}')

    def _arm(self):
        self.get_logger().info('Arming motors (COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM)...')
        # param1=1 => arm, param1=0 => disarm
        self._send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)

    def _disarm(self):
        self.get_logger().info('Disarming motors...')
        self._send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0)

    def _takeoff(self, altitude_m):
        self.get_logger().info(f'Requesting takeoff to {altitude_m} meters...')
        
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param7=float(altitude_m))

    def _land(self):
        self.get_logger().info('Requesting LAND...')
       
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)

    def _goto(self, lat, lon, alt):
        self.get_logger().info(f'Sending GOTO -> lat:{lat} lon:{lon} alt:{alt}')
        
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, param5=float(lat), param6=float(lon), param7=float(alt))

    def _set_mode(self, mode_str):
        self.get_logger().info(f'Setting mode to {mode_str} (best-effort)')
        try:
            self._send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, param1=1, param2=0)
            self.get_logger().warning('Mode set attempt sent — ensure mode mapping is correct for your autopilot.')
        except Exception as e:
            self.get_logger().error(f'Error attempting to set mode: {e}')

    # --- Telemetry publisher ---
    def _telemetry_loop(self):
        rate = 1.0 / max(0.001, self.telemetry_rate)
        while not self._stop and rclpy.ok():
            try:
                # Read a few messages non-blocking and gather telemetry
                msg = self.mav.recv_match(blocking=False)
                telemetry = {}
                if msg is None:
                    # publish heartbeat-summary only
                    try:
                        hb = self.mav.recv_match(type='HEARTBEAT', blocking=False)
                        if hb:
                            telemetry['heartbeat'] = {
                                'type': int(hb.type),
                                'autopilot': int(hb.autopilot),
                                'base_mode': int(hb.base_mode),
                                'custom_mode': int(hb.custom_mode)
                            }
                    except Exception:
                        pass
                else:
                    ttype = msg.get_type()
                    if ttype == 'SYS_STATUS':
                        telemetry['battery'] = {
                            'voltage_battery': getattr(msg, 'voltage_battery', None),
                            'current_battery': getattr(msg, 'current_battery', None),
                            'battery_remaining': getattr(msg, 'battery_remaining', None)
                        }
                    elif ttype == 'GLOBAL_POSITION_INT':
                        telemetry['position'] = {
                            'lat': getattr(msg, 'lat', None) / 1e7 if getattr(msg, 'lat', None) is not None else None,
                            'lon': getattr(msg, 'lon', None) / 1e7 if getattr(msg, 'lon', None) is not None else None,
                            'alt': getattr(msg, 'relative_alt', None) / 1000.0 if getattr(msg, 'relative_alt', None) is not None else None
                        }
                    elif ttype == 'HEARTBEAT':
                        telemetry['heartbeat'] = {
                            'type': int(msg.type),
                            'autopilot': int(msg.autopilot),
                            'base_mode': int(msg.base_mode),
                            'custom_mode': int(msg.custom_mode)
                        }
                    # you can add more message types as needed

                if telemetry:
                    out = String()
                    out.data = json.dumps({
                        'timestamp_ns': int(time.time() * 1e9),
                        'telemetry': telemetry
                    })
                    self.telemetry_pub.publish(out)

            except Exception as e:
                self.get_logger().warning(f'Telemetry loop exception: {e}')
            time.sleep(rate)

    def destroy_node(self):
        self._stop = True
        try:
            if self.telemetry_thread.is_alive():
                self.telemetry_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.mav.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommandBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
