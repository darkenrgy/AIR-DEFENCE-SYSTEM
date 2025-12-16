import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import time
import threading
from typing import Dict, List, Tuple
from datetime import datetime
from pathlib import Path

# Folium for interactive maps
try:
    import folium
    from folium.plugins import MarkerCluster, HeatMap, MiniMap, Fullscreen
    FOLIUM_AVAILABLE = True
except ImportError:
    print(" Folium not installed!")
    print("Install: pip install folium")
    FOLIUM_AVAILABLE = False

import numpy as np




# Output path - will be created automatically
MAP_PATH = "mayuri/ui_control/live_map.html"

# Default map center (Bangalore, India)
DEFAULT_CENTER = [12.9716, 77.5946]




class MapVisualizer(Node):
    """
    Advanced map visualization for MAYURI surveillance drone
    Creates interactive HTML map with real-time updates
    """
    
    def __init__(self):
        super().__init__("map_visualizer")
        
        if not FOLIUM_AVAILABLE:
            self.get_logger().error(" Cannot start: Folium not available")
            return
        
       
        
        self.declare_parameter("update_rate", 1.0)  # Hz
        self.declare_parameter("map_center", DEFAULT_CENTER)
        self.declare_parameter("zoom_start", 17)
        self.declare_parameter("output_path", MAP_PATH)
        self.declare_parameter("auto_center", True)
        self.declare_parameter("show_path_trail", True)
        self.declare_parameter("path_length", 100)
        self.declare_parameter("save_screenshots", False)
        
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.map_center = list(self.get_parameter("map_center").value)
        self.zoom_start = int(self.get_parameter("zoom_start").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.auto_center = bool(self.get_parameter("auto_center").value)
        self.show_path_trail = bool(self.get_parameter("show_path_trail").value)
        self.path_length = int(self.get_parameter("path_length").value)
        self.save_screenshots = bool(self.get_parameter("save_screenshots").value)
        
        # Setup file paths
        self.setup_paths()
        
       
        
        # Drone state
        self.drone_position: Dict[str, float] = {
            "lat": None, 
            "lon": None, 
            "alt": None, 
            "heading": 0
        }
        self.drone_velocity: Dict[str, float] = {
            "vx": 0, 
            "vy": 0, 
            "vz": 0
        }
        
        # Flight path history (with timestamps)
        self.flight_path: List[Tuple[float, float, datetime]] = []
        
        # Mission planning
        self.mission_points: List[Dict] = []
        
        # AI detections
        self.detected_objects: List[Dict] = []
        self.threat_zones: List[Dict] = []
        
        # Telemetry
        self.drone_mode = "UNKNOWN"
        self.battery = 0
        self.armed = False
        self.gps_satellites = 0
        self.ground_speed = 0.0
        self.gps_ok = False
        
        # Status flags
        self.data_received = False
        self.map_generated = False
        self.error_count = 0
        
        # Thread safety
        self.map_lock = threading.Lock()
        
       
        
        self.create_subscription(
            String, 
            "/mayuri/flight/status", 
            self.on_flight_status, 
            10
        )
        
        self.create_subscription(
            String, 
            "/mayuri/mission/waypoints", 
            self.on_mission_waypoints, 
            10
        )
        
        self.create_subscription(
            String, 
            "/mayuri/ai/detections", 
            self.on_ai_detections, 
            10
        )
        
        
        
        # Generate initial map immediately
        self.get_logger().info(" Generating initial map...")
        try:
            self.generate_map()
            if self.output_path_abs.exists():
                size = self.output_path_abs.stat().st_size
                self.get_logger().info(f"✓ Initial map created: {size} bytes")
                self.map_generated = True
            else:
                self.get_logger().error(" Initial map creation failed!")
        except Exception as e:
            self.get_logger().error(f" Map generation error: {e}")
            import traceback
            traceback.print_exc()
        
        # Start map update thread
        self.running = True
        self.map_thread = threading.Thread(target=self.map_loop, daemon=True)
        self.map_thread.start()
        
        # Log startup info
        self.get_logger().info("=" * 60)
        self.get_logger().info(" MAYURI Map Visualizer Initialized")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"   Output: {self.output_path_abs}")
        self.get_logger().info(f"   Update rate: {self.update_rate} Hz")
        self.get_logger().info(f"   Auto-center: {self.auto_center}")
        self.get_logger().info(f"   Path trail: {self.show_path_trail}")
        self.get_logger().info(f"   Zoom level: {self.zoom_start}")
        self.get_logger().info("=" * 60)
    
    
    
    
    def setup_paths(self):
        """Setup and validate output paths"""
        
        # Try multiple possible base paths
        possible_bases = [
            Path.cwd(),
            Path.home() / "mayuri_ws" / "src" / "mayuri",
            Path(__file__).parent.parent.parent,
        ]
        
        base_path = None
        for path in possible_bases:
            if path.exists():
                base_path = path
                break
        
        if base_path is None:
            base_path = Path.cwd()
            self.get_logger().warn(f"Using current directory: {base_path}")
        
        # Create absolute path
        if self.output_path.startswith('/'):
            self.output_path_abs = Path(self.output_path)
        else:
            self.output_path_abs = base_path / self.output_path
        
        # Ensure directory exists
        self.output_path_abs.parent.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f" Map directory: {self.output_path_abs.parent}")
        self.get_logger().info(f" Map file: {self.output_path_abs.name}")
        
        # Test write permissions
        try:
            test_file = self.output_path_abs.parent / ".test_write"
            test_file.write_text("test")
            test_file.unlink()
            self.get_logger().info("✓ Write permissions OK")
        except Exception as e:
            self.get_logger().error(f" Write permission error: {e}")
    
    
   
    
    def on_flight_status(self, msg: String):
        """Process drone flight status updates"""
        try:
            data = json.loads(msg.data)
            pos = data.get("position", {})
            
            if pos.get("lat") and pos.get("lon"):
                self.data_received = True
                
                lat = float(pos["lat"])
                lon = float(pos["lon"])
                alt = float(pos.get("alt", 0.0))
                heading = float(data.get("heading", 0))
                
                # Update current position
                self.drone_position = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "heading": heading
                }
                
                # Add to flight path with timestamp
                current_time = datetime.now()
                
                # Only add if position changed significantly
                if len(self.flight_path) == 0 or \
                   (abs(self.flight_path[-1][0] - lat) > 0.000001 or 
                    abs(self.flight_path[-1][1] - lon) > 0.000001):
                    
                    self.flight_path.append((lat, lon, current_time))
                    
                    # Maintain path length limit
                    if len(self.flight_path) > self.path_length:
                        self.flight_path.pop(0)
                
                # Update telemetry
                vel = data.get("velocity", {})
                self.drone_velocity = {
                    "vx": vel.get("vx", 0),
                    "vy": vel.get("vy", 0),
                    "vz": vel.get("vz", 0)
                }
                
                self.drone_mode = data.get("mode", "UNKNOWN")
                self.battery = int(data.get("battery", {}).get("battery_remaining", 0))
                self.armed = bool(data.get("armed", False))
                self.gps_satellites = int(data.get("gps_satellites", 0))
                self.gps_ok = bool(data.get("gps_ok", False))
                
                # Calculate ground speed
                vx = self.drone_velocity["vx"]
                vy = self.drone_velocity["vy"]
                self.ground_speed = np.sqrt(vx**2 + vy**2)
                
                # Auto-center map on drone
                if self.auto_center:
                    self.map_center = [lat, lon]
                
                # Log position periodically
                self.get_logger().info(
                    f" Drone: {lat:.6f}, {lon:.6f}, {alt:.1f}m, "
                    f"Mode: {self.drone_mode}, Battery: {self.battery}%",
                    throttle_duration_sec=5.0
                )
        
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"Invalid JSON in flight status: {e}")
        except Exception as e:
            self.get_logger().error(f"Flight status error: {e}")
    
    
    def on_mission_waypoints(self, msg: String):
        """Process mission waypoint updates"""
        try:
            data = json.loads(msg.data)
            waypoints = data.get("waypoints", [])
            
            if waypoints:
                self.mission_points = waypoints
                self.get_logger().info(f" Received {len(waypoints)} waypoints")
        
        except Exception as e:
            self.get_logger().warning(f"Waypoint parse error: {e}")
    
    
    def on_ai_detections(self, msg: String):
        """Process AI detection updates"""
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            
            self.detected_objects = []
            
            # Convert detections to GPS coordinates
            if self.drone_position["lat"] and self.drone_position["lon"]:
                for d in detections:
                    # If detection already has GPS, use it
                    if d.get("lat") and d.get("lon"):
                        det_lat = d["lat"]
                        det_lon = d["lon"]
                    else:
                        # Otherwise calculate from drone position
                        # Simple offset for demo - real system needs gimbal angles
                        lat_offset = np.random.uniform(-0.0002, 0.0002)
                        lon_offset = np.random.uniform(-0.0002, 0.0002)
                        det_lat = self.drone_position["lat"] + lat_offset
                        det_lon = self.drone_position["lon"] + lon_offset
                    
                    self.detected_objects.append({
                        "lat": det_lat,
                        "lon": det_lon,
                        "label": d.get("label", "unknown"),
                        "confidence": d.get("confidence", 0.0),
                        "threat_level": d.get("threat_level", "NORMAL")
                    })
            
            # Create threat zones for high/critical threats
            self.threat_zones = [
                {
                    "lat": obj["lat"],
                    "lon": obj["lon"],
                    "radius": 30
                }
                for obj in self.detected_objects
                if obj.get("threat_level") in ["HIGH", "CRITICAL"]
            ]
            
            if detections:
                self.get_logger().info(
                    f"Detections: {len(detections)}, Threats: {len(self.threat_zones)}",
                    throttle_duration_sec=3.0
                )
        
        except Exception as e:
            self.get_logger().warning(f"Detection parse error: {e}")
    
    
    
    
    def generate_map(self):
        """Generate interactive HTML map"""
        
        with self.map_lock:
            try:
                # Create base map
                m = folium.Map(
                    location=self.map_center,
                    zoom_start=self.zoom_start,
                    tiles=None,
                    control_scale=True,
                    prefer_canvas=True
                )
                
                # ===== TILE LAYERS 
                
                # Google Streets
                folium.TileLayer(
                    tiles='https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',
                    attr='Google',
                    name='Google Streets',
                    overlay=False,
                    control=True
                ).add_to(m)
                
                # Google Satellite
                folium.TileLayer(
                    tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
                    attr='Google',
                    name='Google Satellite',
                    overlay=False,
                    control=True
                ).add_to(m)
                
                # Google Hybrid
                folium.TileLayer(
                    tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
                    attr='Google',
                    name='Google Hybrid',
                    overlay=False,
                    control=True
                ).add_to(m)
                
                # OpenStreetMap
                folium.TileLayer(
                    'OpenStreetMap',
                    name='OpenStreetMap',
                    overlay=False,
                    control=True
                ).add_to(m)
                
                # ===== MAP PLUGINS 
                
                MiniMap(toggle_display=True).add_to(m)
                Fullscreen(position='topleft').add_to(m)
                
                # ===== FEATURE GROUPS 
                
                drone_group = folium.FeatureGroup(name=' Drone Position', show=True)
                path_group = folium.FeatureGroup(name=' Flight Path', show=True)
                mission_group = folium.FeatureGroup(name=' Mission Waypoints', show=True)
                detection_group = folium.FeatureGroup(name=' AI Detections', show=True)
                threat_group = folium.FeatureGroup(name=' Threat Zones', show=True)
                
                # ===== HOME MARKER (ALWAYS VISIBLE)
                
                folium.Marker(
                    location=DEFAULT_CENTER,
                    popup="<b> Home Base</b><br>Bangalore, India",
                    tooltip="Home Location",
                    icon=folium.Icon(color='green', icon='home', prefix='fa')
                ).add_to(m)
                
                # ===== FLIGHT PATH TRAIL 
                
                if self.show_path_trail and len(self.flight_path) > 1:
                    path_coords = [(p[0], p[1]) for p in self.flight_path]
                    
                    # Main flight path line
                    folium.PolyLine(
                        path_coords,
                        color='cyan',
                        weight=3,
                        opacity=0.7,
                        tooltip=f"Flight path ({len(path_coords)} points)"
                    ).add_to(path_group)
                    
                    # Time markers every 20 points
                    for i in range(0, len(self.flight_path), 20):
                        p = self.flight_path[i]
                        timestamp = p[2].strftime("%H:%M:%S")
                        
                        folium.CircleMarker(
                            location=[p[0], p[1]],
                            radius=3,
                            color='cyan',
                            fill=True,
                            fillColor='cyan',
                            fillOpacity=0.6,
                            popup=f"<b>Time:</b> {timestamp}",
                            tooltip=timestamp
                        ).add_to(path_group)
                
                # ===== DRONE POSITION 
                
                if self.drone_position["lat"] and self.drone_position["lon"]:
                    drone_lat = self.drone_position["lat"]
                    drone_lon = self.drone_position["lon"]
                    
                    # Battery color
                    if self.battery > 50:
                        bat_color = "#00ff00"
                    elif self.battery > 20:
                        bat_color = "#ff9900"
                    else:
                        bat_color = "#ff0000"
                    
                    # Telemetry popup
                    popup_html = f"""
                    <div style='width: 300px; font-family: monospace; background: #1a1a1a; color: #fff; padding: 10px; border-radius: 5px;'>
                        <h4 style='margin:0 0 10px 0; color:#00aaff; text-align: center;'>
                             MAYURI Surveillance Drone
                        </h4>
                        <hr style='margin:5px 0; border-color: #444;'>
                        <table style='width:100%; font-size:13px;'>
                            <tr>
                                <td><b>Flight Mode:</b></td>
                                <td style='color:#00ff00;'>{self.drone_mode}</td>
                            </tr>
                            <tr>
                                <td><b>Armed Status:</b></td>
                                <td style='color:{"#00ff00" if self.armed else "#ff0000"};'>
                                    {' ARMED' if self.armed else ' DISARMED'}
                                </td>
                            </tr>
                            <tr>
                                <td><b>Battery:</b></td>
                                <td style='color:{bat_color}; font-weight: bold;'>
                                    {self.battery}%
                                </td>
                            </tr>
                            <tr>
                                <td><b>Altitude:</b></td>
                                <td>{self.drone_position['alt']:.1f} m</td>
                            </tr>
                            <tr>
                                <td><b>Heading:</b></td>
                                <td>{self.drone_position['heading']:.0f}°</td>
                            </tr>
                            <tr>
                                <td><b>Ground Speed:</b></td>
                                <td>{self.ground_speed:.2f} m/s</td>
                            </tr>
                            <tr>
                                <td><b>GPS Satellites:</b></td>
                                <td style='color:{"#00ff00" if self.gps_satellites >= 10 else "#ff9900"};'>
                                    {self.gps_satellites}
                                </td>
                            </tr>
                            <tr>
                                <td><b>GPS Status:</b></td>
                                <td style='color:{"#00ff00" if self.gps_ok else "#ff0000"};'>
                                    {'✓ OK' if self.gps_ok else '✗ DEGRADED'}
                                </td>
                            </tr>
                        </table>
                        <hr style='margin:8px 0; border-color: #444;'>
                        <div style='font-size:11px; color:#888;'>
                            <b>Position:</b><br>
                            Lat: {drone_lat:.6f}<br>
                            Lon: {drone_lon:.6f}
                        </div>
                        <div style='text-align: center; margin-top: 8px; font-size:10px; color:#666;'>
                            Updated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
                        </div>
                    </div>
                    """
                    
                    # Drone marker
                    folium.Marker(
                        [drone_lat, drone_lon],
                        popup=folium.Popup(popup_html, max_width=350),
                        tooltip=f" Drone @ {self.drone_position['alt']:.0f}m | {self.drone_mode}",
                        icon=folium.Icon(
                            color='blue',
                            icon='plane',
                            prefix='fa'
                        )
                    ).add_to(drone_group)
                    
                    # Heading arrow
                    heading_rad = np.radians(self.drone_position['heading'])
                    arrow_length = 0.0001
                    arrow_end_lat = drone_lat + arrow_length * np.cos(heading_rad)
                    arrow_end_lon = drone_lon + arrow_length * np.sin(heading_rad)
                    
                    folium.PolyLine(
                        [[drone_lat, drone_lon], [arrow_end_lat, arrow_end_lon]],
                        color='blue',
                        weight=5,
                        opacity=0.8,
                        tooltip=f"Heading: {self.drone_position['heading']:.0f}°"
                    ).add_to(drone_group)
                    
                    # Speed circle indicator
                    speed_radius = max(10, self.ground_speed * 5)
                    folium.Circle(
                        location=[drone_lat, drone_lon],
                        radius=speed_radius,
                        color='blue',
                        fill=False,
                        weight=2,
                        opacity=0.4,
                        tooltip=f"Speed: {self.ground_speed:.1f} m/s"
                    ).add_to(drone_group)
                
                # ===== MISSION WAYPOINTS 
                
                if self.mission_points:
                    mission_coords = [
                        (wp.get("lat"), wp.get("lon"))
                        for wp in self.mission_points
                        if wp.get("lat") and wp.get("lon")
                    ]
                    
                    if mission_coords:
                        # Mission path line
                        folium.PolyLine(
                            mission_coords,
                            color='orange',
                            weight=4,
                            opacity=0.7,
                            dash_array='10, 5',
                            tooltip="Planned Mission Path"
                        ).add_to(mission_group)
                        
                        # Waypoint markers
                        for i, wp in enumerate(self.mission_points, start=1):
                            if wp.get("lat") and wp.get("lon"):
                                folium.Marker(
                                    [wp["lat"], wp["lon"]],
                                    popup=f"""
                                    <b>Waypoint {i}</b><br>
                                    Altitude: {wp.get('alt', 0):.0f} m<br>
                                    Latitude: {wp['lat']:.6f}<br>
                                    Longitude: {wp['lon']:.6f}
                                    """,
                                    tooltip=f"WP{i} @ {wp.get('alt', 0):.0f}m",
                                    icon=folium.Icon(
                                        color='orange',
                                        icon='flag-checkered',
                                        prefix='fa'
                                    )
                                ).add_to(mission_group)
                
                # ===== AI DETECTIONS
                
                for det in self.detected_objects:
                    lat, lon = det["lat"], det["lon"]
                    label = det["label"]
                    conf = det["confidence"]
                    threat = det.get("threat_level", "NORMAL")
                    
                    # Threat-based color
                    color_map = {
                        "NORMAL": "green",
                        "MEDIUM": "orange",
                        "HIGH": "red",
                        "CRITICAL": "darkred"
                    }
                    color = color_map.get(threat, "gray")
                    
                    # Detection type icon
                    if " " in label or "weapon" in label.lower():
                        icon = "exclamation-triangle"
                    elif "👥" in label or "person" in label.lower():
                        icon = "user"
                    else:
                        icon = "eye"
                    
                    folium.Marker(
                        location=[lat, lon],
                        popup=f"""
                        <b>{label}</b><br>
                        Confidence: {conf*100:.1f}%<br>
                        Threat Level: <span style='color:{color}; font-weight: bold;'>{threat}</span>
                        """,
                        tooltip=f"{label} ({threat})",
                        icon=folium.Icon(
                            color=color,
                            icon=icon,
                            prefix='fa'
                        )
                    ).add_to(detection_group)
                
                # ===== THREAT ZONES 
                
                for zone in self.threat_zones:
                    folium.Circle(
                        location=[zone["lat"], zone["lon"]],
                        radius=zone["radius"],
                        color='red',
                        fill=True,
                        fillColor='red',
                        fillOpacity=0.2,
                        weight=2,
                        popup="<b> High Threat Zone</b><br>Approach with caution",
                        tooltip="Threat Zone"
                    ).add_to(threat_group)
                
                # ===== DETECTION HEATMAP =
                
                if len(self.detected_objects) > 3:
                    heat_data = [
                        [obj["lat"], obj["lon"], obj["confidence"]]
                        for obj in self.detected_objects
                    ]
                    
                    HeatMap(
                        heat_data,
                        name=' Detection Heatmap',
                        min_opacity=0.3,
                        radius=25,
                        blur=20,
                        gradient={
                            0.4: 'blue',
                            0.65: 'lime',
                            0.8: 'yellow',
                            1.0: 'red'
                        },
                        show=True
                    ).add_to(m)
                
                # ===== ADD GROUPS TO MAP 
                
                drone_group.add_to(m)
                path_group.add_to(m)
                mission_group.add_to(m)
                detection_group.add_to(m)
                threat_group.add_to(m)
                
                # Layer control
                folium.LayerControl(
                    position='topright',
                    collapsed=False
                ).add_to(m)
                
                # ===== CUSTOM STYLING 
                
                m.get_root().html.add_child(folium.Element("""
                <style>
                    .leaflet-container {
                        background: #0a0a0a;
                        font-family: 'Courier New', monospace;
                    }
                    .leaflet-popup-content-wrapper {
                        background: #1a1a1a;
                        color: #fff;
                        border: 2px solid #00aaff;
                        border-radius: 8px;
                    }
                    .leaflet-popup-tip {
                        background: #1a1a1a;
                    }
                    .leaflet-control-layers {
                        background: rgba(0, 0, 0, 0.8);
                        color: #fff;
                    }
                </style>
                """))
                
                # ===== STATUS INDICATOR 
                
                status_color = "#00ff00" if self.data_received else "#ff9900"
                status_text = "✓ LIVE" if self.data_received else "WAITING"
                
                status_html = f"""
                <div style="position: fixed; bottom: 10px; left: 10px; z-index: 1000; 
                            background: rgba(0,0,0,0.9); color: #0f0; padding: 12px; 
                            border-radius: 8px; font-family: monospace; font-size: 12px;
                            border: 2px solid #00aaff; min-width: 280px;">
                    <div style="font-weight: bold; font-size: 14px; margin-bottom: 5px;">
                         MAYURI Map Visualizer
                    </div>
                    <div style="margin: 3px 0;">
                         Center: {self.map_center[0]:.4f}, {self.map_center[1]:.4f}
                    </div>
                    <div style="margin: 3px 0;">
                         {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
                    </div>
                    <div style="margin: 3px 0;">
                         Status: <span style="color: {status_color}; font-weight: bold;">{status_text}</span>
                    </div>
                    <div style="margin: 3px 0;">
                         Path Points: {len(self.flight_path)}
                    </div>
                    <div style="margin: 3px 0;">
                        Detections: {len(self.detected_objects)}
                    </div>
                </div>
                """
                
                m.get_root().html.add_child(folium.Element(status_html))
                
                # ===== SAVE MAP 
                
                m.save(str(self.output_path_abs))
                
                # Verify file creation
                if self.output_path_abs.exists():
                    file_size = self.output_path_abs.stat().st_size
                    
                    if not self.map_generated:
                        self.get_logger().info(f"✓ Map generated: {file_size} bytes")
                        self.map_generated = True
                    
                    self.error_count = 0  # Reset error count on success
                else:
                    self.get_logger().error(" Map file not found after save!")
                    self.error_count += 1
            
            except Exception as e:
                self.error_count += 1
                self.get_logger().error(f" Map generation error ({self.error_count}): {e}")
                
                if self.error_count <= 3:
                    import traceback
                    traceback.print_exc()
    
    
   
    
    def map_loop(self):
        """Continuously update map at specified rate"""
        
        self.get_logger().info(" Map update loop started")
        
        update_interval = 1.0 / max(0.1, self.update_rate)
        
        while self.running and rclpy.ok():
            try:
                self.generate_map()
            except Exception as e:
                self.get_logger().error(f"Map loop error: {e}")
            
            time.sleep(update_interval)
        
        self.get_logger().info(" Map update loop stopped")
    
    
   
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info(" Shutting down map visualizer...")
        
        self.running = False
        
        if self.map_thread.is_alive():
            self.map_thread.join(timeout=2.0)
        
        self.get_logger().info("✓ Map visualizer shutdown complete")
        
        super().destroy_node()




def main(args=None):
    """Main entry point for map visualizer"""
    
    if not FOLIUM_AVAILABLE:
        print("=" * 70)
        print(" FOLIUM NOT INSTALLED")
        print("=" * 70)
        print("\nInstall with:")
        print("  pip install folium")
        print("\nOr:")
        print("  pip3 install folium")
        print("\n" + "=" * 70)
        return
    
    rclpy.init(args=args)
    node = None
    
    try:
        node = MapVisualizer()
        
        if node and node.map_generated:
            print("\n" + "=" * 70)
            print(" MAYURI MAP VISUALIZER RUNNING")
            print("=" * 70)
            print(f"\nMap file: {node.output_path_abs}")
            print(f"\nOpen in browser:")
            print(f"  file://{node.output_path_abs.absolute()}")
            print(f"\nOr in dashboard:")
            print(f"  http://localhost:8050")
            print("\n" + "=" * 70)
            print("\nPress Ctrl+C to stop\n")
            
            rclpy.spin(node)
    
    except KeyboardInterrupt:
        print("\n\n Stopping map visualizer...")
    
    except Exception as e:
        print(f"\n Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if node:
            node.destroy_node()
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("✓ Shutdown complete\n")


if __name__ == "__main__":
    main()
