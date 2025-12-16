import json
import threading
import time
import os
from pathlib import Path
from datetime import datetime

# Dash imports
import dash
from dash import html, dcc, Input, Output, State, callback_context
from dash.exceptions import PreventUpdate
import plotly.graph_objects as go
import dash_bootstrap_components as dbc

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Flask imports
from flask import send_from_directory

# NumPy for calculations
import numpy as np




DASHBOARD_PORT = 8050
UPDATE_INTERVAL = 1000  # milliseconds
CAMERA_UPDATE_INTERVAL = 100  # milliseconds (10 FPS)

# Setup paths
WORKSPACE_ROOT = Path.home() / "mayuri_ws"
STATIC_DIR = Path("mayuri/ui_control")
ASSETS_DIR = STATIC_DIR / "assets"

# Ensure directories exist
STATIC_DIR.mkdir(parents=True, exist_ok=True)
ASSETS_DIR.mkdir(parents=True, exist_ok=True)

# Threat colors
THREAT_COLORS = {
    'NORMAL': '#28a745',
    'MEDIUM': '#ffc107',
    'HIGH': '#fd7e14',
    'CRITICAL': '#dc3545'
}




class DashboardNode(Node):
    """ROS2 node for dashboard communication with MAYURI system"""
    
    def __init__(self):
        super().__init__("dashboard_ui_node")
        
        # State variables
        self.flight_status = {}
        self.ai_detections = []
        self.last_event = ""
        self.threat_level = "NORMAL"
        self.threat_history = []
        self.is_connected = False
        
        # Camera stream
        self.camera_frame = None
        self.camera_timestamp = None
        self.camera_threat_level = "NORMAL"
        
        # Statistics
        self.message_count = {
            'flight_status': 0,
            'ai_detections': 0,
            'events': 0,
            'camera': 0
        }
        
        # Publishers
        self.cmd_pub = self.create_publisher(String, "/mayuri/flight/cmd", 10)
        
        # Subscribers
        self.create_subscription(String, "/mayuri/flight/status", self._on_status, 10)
        self.create_subscription(String, "/mayuri/ai/detections", self._on_ai, 10)
        self.create_subscription(String, "/mayuri/failsafe/event", self._on_event, 10)
        self.create_subscription(String, "/mayuri/camera/stream", self._on_camera, 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("MAYURI Dashboard Node Initialized")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Subscribed to:")
        self.get_logger().info("   - /mayuri/flight/status")
        self.get_logger().info("   - /mayuri/ai/detections")
        self.get_logger().info("   - /mayuri/failsafe/event")
        self.get_logger().info("   - /mayuri/camera/stream")
        self.get_logger().info("Publishing to:")
        self.get_logger().info("   - /mayuri/flight/cmd")
        self.get_logger().info("=" * 60)
    
    
    def _on_status(self, msg: String):
        """Handle flight status updates"""
        try:
            data = json.loads(msg.data)
            self.flight_status = data
            self.is_connected = True
            self.message_count['flight_status'] += 1
            
            if self.message_count['flight_status'] % 10 == 0:
                self.get_logger().info(
                    f"Status updates: {self.message_count['flight_status']}",
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            self.get_logger().warning(f"Status parse error: {e}")
    
    
    def _on_ai(self, msg: String):
        """Handle AI detection updates"""
        try:
            data = json.loads(msg.data)
            self.ai_detections = data.get("detections", [])
            self.threat_level = data.get("overall_threat", "NORMAL")
            self.message_count['ai_detections'] += 1
            
            # Maintain threat history
            if len(self.threat_history) >= 100:
                self.threat_history.pop(0)
            
            self.threat_history.append({
                'timestamp': time.time(),
                'level': self.threat_level,
                'count': len(self.ai_detections)
            })
            
        except Exception as e:
            self.get_logger().warning(f"AI detection parse error: {e}")
    
    
    def _on_event(self, msg: String):
        """Handle failsafe/event updates"""
        try:
            event = json.loads(msg.data)
            reason = event.get('reason', 'Unknown')
            action = event.get('action', '')
            self.last_event = f"{reason}: {action}"
            self.message_count['events'] += 1
            self.get_logger().warn(f"Event: {self.last_event}")
        except Exception as e:
            self.get_logger().warning(f"Event parse error: {e}")
    
    
    def _on_camera(self, msg: String):
        """Handle camera stream updates"""
        try:
            data = json.loads(msg.data)
            self.camera_frame = data.get("image", None)
            self.camera_timestamp = data.get("timestamp", datetime.now().isoformat())
            self.camera_threat_level = data.get("threat_level", "NORMAL")
            self.message_count['camera'] += 1
        except Exception as e:
            self.get_logger().warning(f"Camera parse error: {e}")
    
    
    def send_command(self, command: dict):
        """Send flight command"""
        try:
            msg = String()
            msg.data = json.dumps(command)
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Command sent: {command}")
            return True
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")
            return False
    
    
    def get_stats(self):
        """Get connection statistics"""
        return {
            'connected': self.is_connected,
            'messages': self.message_count,
            'threat_level': self.threat_level,
            'detections': len(self.ai_detections)
        }




node = None
ros_thread_running = False

def start_ros():
    """Start ROS2 node in background thread"""
    global node, ros_thread_running
    
    try:
        rclpy.init()
        node = DashboardNode()
        ros_thread_running = True
        
        while rclpy.ok() and ros_thread_running:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f" ROS2 thread error: {e}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

# Start ROS2 thread
ros_thread = threading.Thread(target=start_ros, daemon=True)
ros_thread.start()
time.sleep(2)  # Wait for initialization




# Initialize app
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.CYBORG],
    assets_folder=str(ASSETS_DIR),
    suppress_callback_exceptions=True
)
app.title = "MAYURI Surveillance Dashboard"




@app.server.route('/static/<path:filename>')
def serve_static(filename):
    """Serve static files (map/camera HTML) - FIXED VERSION"""
    
    # Check multiple possible locations
    possible_paths = [
        STATIC_DIR / filename,
        Path("mayuri/ui_control") / filename,
        Path.cwd() / "mayuri/ui_control" / filename,
        WORKSPACE_ROOT / "mayuri/ui_control" / filename,
    ]
    
    for path in possible_paths:
        if path.exists():
            print(f"✓ Serving: {path}")
            return send_from_directory(str(path.parent), path.name)
    
    # Helpful error page
    error_html = f"""
    <html>
    <head>
        <title>File Not Found - MAYURI Dashboard</title>
        <style>
            body {{
                background: #0a0a0a;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                padding: 40px;
                line-height: 1.6;
            }}
            h2 {{ color: #ff9900; }}
            h3 {{ color: #00aaff; }}
            .path {{ background: #1a1a1a; padding: 5px; border-radius: 3px; }}
            .exists {{ color: #00ff00; }}
            .missing {{ color: #ff0000; }}
            pre {{ background: #000; padding: 10px; border-radius: 5px; border: 1px solid #00ff00; }}
        </style>
    </head>
    <body>
        <h2> Map File Not Found</h2>
        <p><strong>Requested file:</strong> <span class="path">{filename}</span></p>
        
        <h3> Searched in these locations:</h3>
        <ul>
            {''.join([
                f'<li><span class="{"exists" if p.exists() else "missing"}">{"✓" if p.exists() else "✗"}</span> {p}</li>'
                for p in possible_paths
            ])}
        </ul>
        
        <h3>🔧 Solution:</h3>
        <ol>
            <li><strong>Start map visualizer first:</strong>
                <pre>cd ~/mayuri_ws
python3 src/mayuri/mayuri/ui_control/map_visualizer.py</pre>
            </li>
            <li>Wait for message: <code>✓ Initial map created</code></li>
            <li>Verify file exists:
                <pre>ls -lh mayuri/ui_control/live_map.html</pre>
            </li>
            <li>Refresh this page (F5)</li>
        </ol>
        
        <p style="margin-top: 30px; color: #888;">
            <strong>Current directory:</strong> {Path.cwd()}<br>
            <strong>Time:</strong> {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        </p>
    </body>
    </html>
    """
    return error_html, 404


#DAshbossard      layout

app.layout = dbc.Container([
    
    # ===== HEADER 
    dbc.Row([
        dbc.Col([
            html.H2(
                [
                    html.I(className="fas fa-helicopter me-3"),
                    "MAYURI  AI-Driven Surveillance & Defense Drone"
                ],
                className="text-center mt-3 mb-3",
                style={
                    "font-weight": "bold",
                    "text-shadow": "2px 2px 4px rgba(0,0,0,0.5)",
                    "color": "#00aaff"
                }
            )
        ])
    ]),
    
    # ===== THREAT LEVEL BANNER 
    dbc.Row([
        dbc.Col([
            html.Div(
                id="threat-banner",
                className="text-center p-3 mb-3",
                style={
                    "border-radius": "10px",
                    "font-size": "24px",
                    "font-weight": "bold",
                    "box-shadow": "0 4px 8px rgba(0,0,0,0.4)"
                }
            )
        ], width=12)
    ]),
    
    # ===== CONNECTION STATUS 
    dbc.Row([
        dbc.Col([
            html.Div(id="connection-status", className="text-center mb-2")
        ], width=12)
    ]),
    
    # ===== MAIN CONTENT ROW 
    dbc.Row([
        
        # DRONE STATUS CARD
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.I(className="fas fa-tachometer-alt me-2"),
                    "Drone Telemetry"
                ], style={"font-weight": "bold", "font-size": "16px"}),
                dbc.CardBody([
                    html.Div(id="status-text", style={"min-height": "200px", "font-size": "14px"}),
                    html.Hr(),
                    html.Div(
                        id="event-text",
                        className="text-warning fw-bold",
                        style={"min-height": "40px", "font-size": "13px"}
                    )
                ])
            ], style={"height": "100%", "box-shadow": "0 4px 6px rgba(0,0,0,0.3)"})
        ], width=4),
        
        # VISUALIZATION CARD
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.Div([
                        html.Span([
                            html.I(id="view-icon", className="fas fa-map-marked-alt me-2"),
                            html.Span("Visualization", id="view-title")
                        ], style={"font-weight": "bold", "font-size": "16px"}),
                        html.Span([
                            dbc.ButtonGroup([
                                dbc.Button(
                                    [html.I(className="fas fa-map me-1"), "Map"],
                                    id="btn-view-map",
                                    size="sm",
                                    color="primary",
                                    outline=False
                                ),
                                dbc.Button(
                                    [html.I(className="fas fa-video me-1"), "Camera"],
                                    id="btn-view-camera",
                                    size="sm",
                                    color="secondary",
                                    outline=True
                                ),
                            ], size="sm")
                        ], style={"float": "right"})
                    ], className="d-flex justify-content-between align-items-center w-100")
                ]),
                dbc.CardBody([
                    # Map iframe
                    html.Iframe(
                        id="map-frame",
                        src="/static/live_map.html",
                        style={
                            "width": "100%",
                            "height": "450px",
                            "border": "2px solid #444",
                            "border-radius": "5px",
                            "display": "block"
                        }
                    ),
                    # Camera container (hidden by default)
                    html.Div(
                        id="camera-container",
                        style={"display": "none"},
                        children=[
                            html.Img(
                                id="camera-feed",
                                style={
                                    "width": "100%",
                                    "height": "450px",
                                    "border": "3px solid #00ff00",
                                    "border-radius": "5px",
                                    "object-fit": "contain",
                                    "background": "#000"
                                }
                            ),
                            html.Div(
                                id="camera-status",
                                className="text-center mt-2",
                                style={"color": "#00ff00", "font-size": "12px"}
                            )
                        ]
                    )
                ], style={"padding": "10px", "background": "#0a0a0a"})
            ], style={"box-shadow": "0 4px 6px rgba(0,0,0,0.3)"})
        ], width=8),
        
    ], className="mb-3"),
    
    # ===== DETECTION GRAPHS ROW 
    dbc.Row([
        
        # AI THREAT DETECTIONS
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.I(className="fas fa-crosshairs me-2"),
                    "AI Threat Detections"
                ], style={"font-weight": "bold", "font-size": "16px"}),
                dbc.CardBody([
                    dcc.Graph(
                        id="ai-detection-graph",
                        config={"displayModeBar": False},
                        style={"height": "300px"}
                    )
                ])
            ], style={"box-shadow": "0 4px 6px rgba(0,0,0,0.3)"})
        ], width=6),
        
        # THREAT LEVEL HISTORY
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.I(className="fas fa-chart-line me-2"),
                    "Threat Level History"
                ], style={"font-weight": "bold", "font-size": "16px"}),
                dbc.CardBody([
                    dcc.Graph(
                        id="threat-history-graph",
                        config={"displayModeBar": False},
                        style={"height": "300px"}
                    )
                ])
            ], style={"box-shadow": "0 4px 6px rgba(0,0,0,0.3)"})
        ], width=6),
        
    ], className="mb-3"),
    
    # ===== FLIGHT CONTROLS ROW 
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.I(className="fas fa-gamepad me-2"),
                    "Flight Controls"
                ], style={"font-weight": "bold", "font-size": "16px"}),
                dbc.CardBody([
                    dbc.ButtonGroup([
                        dbc.Button(
                            [html.I(className="fas fa-power-off me-2"), "Arm"],
                            id="btn-arm",
                            color="success",
                            size="lg"
                        ),
                        dbc.Button(
                            [html.I(className="fas fa-ban me-2"), "Disarm"],
                            id="btn-disarm",
                            color="danger",
                            size="lg"
                        ),
                        dbc.Button(
                            [html.I(className="fas fa-plane-departure me-2"), "Takeoff"],
                            id="btn-takeoff",
                            color="primary",
                            size="lg"
                        ),
                        dbc.Button(
                            [html.I(className="fas fa-plane-arrival me-2"), "Land"],
                            id="btn-land",
                            color="secondary",
                            size="lg"
                        ),
                        dbc.Button(
                            [html.I(className="fas fa-pause me-2"), "Hold"],
                            id="btn-hold",
                            color="warning",
                            size="lg"
                        ),
                    ], className="d-flex justify-content-center w-100"),
                    html.Hr(),
                    html.Div(
                        id="detection-count",
                        className="fs-5 fw-bold text-center mt-2",
                        style={"min-height": "35px"}
                    )
                ])
            ], style={"box-shadow": "0 4px 6px rgba(0,0,0,0.3)"})
        ], width=12)
    ]),
    
    # ===== HIDDEN STORAGE & INTERVALS =====
    dcc.Store(id='view-mode', data='map'),
    dcc.Interval(id="update-interval", interval=UPDATE_INTERVAL, n_intervals=0),
    dcc.Interval(id="camera-interval", interval=CAMERA_UPDATE_INTERVAL, n_intervals=0),
    
], fluid=True, style={"padding": "20px", "background": "#0f0f0f", "min-height": "100vh"})




@app.callback(
    Output('view-mode', 'data'),
    Output('btn-view-map', 'outline'),
    Output('btn-view-camera', 'outline'),
    Output('map-frame', 'style'),
    Output('camera-container', 'style'),
    Output('view-icon', 'className'),
    Input('btn-view-map', 'n_clicks'),
    Input('btn-view-camera', 'n_clicks'),
    State('view-mode', 'data')
)
def toggle_view(map_clicks, camera_clicks, current_mode):
    """Toggle between map and camera views"""
    
    ctx = callback_context
    
    if not ctx.triggered:
        return 'map', False, True, \
               {"width": "100%", "height": "450px", "border": "2px solid #444", "border-radius": "5px", "display": "block"}, \
               {"display": "none"}, \
               "fas fa-map-marked-alt me-2"
    
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    
    if button_id == 'btn-view-map':
        return 'map', False, True, \
               {"width": "100%", "height": "450px", "border": "2px solid #444", "border-radius": "5px", "display": "block"}, \
               {"display": "none"}, \
               "fas fa-map-marked-alt me-2"
    elif button_id == 'btn-view-camera':
        return 'camera', True, False, \
               {"display": "none"}, \
               {"display": "block"}, \
               "fas fa-video me-2"
    
    return current_mode, False if current_mode == 'map' else True, True if current_mode == 'map' else False, \
           {"width": "100%", "height": "450px", "border": "2px solid #444", "display": "block" if current_mode == 'map' else "none"}, \
           {"display": "none" if current_mode == 'map' else "block"}, \
           "fas fa-map-marked-alt me-2" if current_mode == 'map' else "fas fa-video me-2"




@app.callback(
    Output('camera-feed', 'src'),
    Output('camera-status', 'children'),
    Input('camera-interval', 'n_intervals'),
    State('view-mode', 'data')
)
def update_camera_feed(n, view_mode):
    """Update camera feed"""
    
    if view_mode != 'camera':
        raise PreventUpdate
    
    if not node or not node.camera_frame:
        return "", html.Span(" Waiting for camera stream...", style={"color": "#888"})
    
    img_src = f"data:image/jpeg;base64,{node.camera_frame}"
    
    color = THREAT_COLORS.get(node.camera_threat_level, '#888')
    status_text = html.Span([
        " LIVE | ",
        html.Span(f"Threat: {node.camera_threat_level}", style={"color": color, "font-weight": "bold"}),
        f" | {node.camera_timestamp}"
    ], style={"color": "#00ff00"})
    
    return img_src, status_text


# ===== CONNECTION STATUS 

@app.callback(
    Output("connection-status", "children"),
    Input("update-interval", "n_intervals"),
)
def update_connection_status(n):
    """Show connection status"""
    
    if not node:
        return html.Div([
            html.I(className="fas fa-spinner fa-spin me-2"),
            html.Span(" Initializing ROS2 connection...", style={"color": "#ffc107"})
        ])
    
    stats = node.get_stats()
    
    if stats['connected']:
        return html.Div([
            html.I(className="fas fa-check-circle me-2", style={"color": "#00ff00"}),
            html.Span(f"✓ Connected | Messages: {sum(stats['messages'].values())}", 
                     style={"color": "#00ff00", "font-size": "14px"})
        ])
    else:
        return html.Div([
            html.I(className="fas fa-exclamation-triangle me-2", style={"color": "#ff9900"}),
            html.Span("⏳ Waiting for drone data...", style={"color": "#ff9900", "font-size": "14px"})
        ])


# ===== THREAT BANNER 

@app.callback(
    Output("threat-banner", "children"),
    Output("threat-banner", "style"),
    Input("update-interval", "n_intervals"),
)
def update_threat_banner(n):
    """Update threat level banner"""
    
    if not node:
        return " SYSTEM INITIALIZING...", {
            "background-color": "#6c757d",
            "color": "white",
            "border-radius": "10px",
            "padding": "15px",
            "font-size": "24px",
            "font-weight": "bold"
        }
    
    level = node.threat_level
    color = THREAT_COLORS.get(level, '#6c757d')
    
    icons = {"NORMAL": "'#6c757d' ", "MEDIUM": " ", "HIGH": " ", "CRITICAL": " "}
    icon = icons.get(level, " ")
    
    style = {
        "background-color": color,
        "color": "white",
        "border-radius": "10px",
        "padding": "15px",
        "font-size": "24px",
        "font-weight": "bold",
        "box-shadow": "0 4px 8px rgba(0,0,0,0.4)"
    }
    
    return f"{icon} THREAT LEVEL: {level}", style


# ===== DRONE STATUS 

@app.callback(
    Output("status-text", "children"),
    Output("event-text", "children"),
    Input("update-interval", "n_intervals"),
)
def update_status(n):
    """Update drone status display"""
    
    if not node or not node.flight_status:
        return [
            html.P([
                html.I(className="fas fa-spinner fa-spin me-2"),
                " Waiting for telemetry..."
            ], className="text-muted")
        ], ""
    
    try:
        pos = node.flight_status.get("position", {})
        mode = node.flight_status.get("mode", "UNKNOWN")
        armed = node.flight_status.get("armed", False)
        battery = node.flight_status.get("battery", {}).get("battery_remaining", 0)
        
        bat_color = "text-success" if battery > 50 else "text-warning" if battery > 20 else "text-danger"
        armed_color = "text-success" if armed else "text-muted"
        
        status = [
            html.P([
                html.I(className="fas fa-cog me-2"),
                html.Strong("Mode: "),
                html.Span(mode, className="text-info fw-bold")
            ], className="mb-2"),
            html.P([
                html.I(className="fas fa-battery-three-quarters me-2"),
                html.Strong("Battery: "),
                html.Span(f"{battery}%", className=f"{bat_color} fw-bold")
            ], className="mb-2"),
            html.P([
                html.I(className="fas fa-shield-alt me-2"),
                html.Strong("Armed: "),
                html.Span(" YES" if armed else " NO", className=f"{armed_color} fw-bold")
            ], className="mb-2"),
            html.P([
                html.I(className="fas fa-arrows-alt-v me-2"),
                html.Strong("Altitude: "),
                html.Span(f"{pos.get('alt', 0):.1f} m", className="text-light")
            ], className="mb-2"),
            html.P([
                html.I(className="fas fa-map-pin me-2"),
                html.Strong("Position: "),
                html.Br(),
                html.Span(f"  {pos.get('lat', 0):.6f}, {pos.get('lon', 0):.6f}",
                         className="text-light",
                         style={"font-size": "0.85em", "font-family": "monospace"})
            ], className="mb-2"),
        ]
        
        return status, node.last_event
    
    except Exception as e:
        return [html.P(f" Error: {str(e)}", className="text-danger")], ""


# ===== AI DETECTION GRAPH

@app.callback(
    Output("ai-detection-graph", "figure"),
    Input("update-interval", "n_intervals"),
)
def update_ai_detections(n):
    """Update AI detection bar chart"""
    
    detections = node.ai_detections if node else []
    
    if not detections:
        fig = go.Figure()
        fig.add_annotation(
            text=" No threats detected",
            xref="paper", yref="paper",
            x=0.5, y=0.5,
            showarrow=False,
            font=dict(size=20, color="#28a745")
        )
        fig.update_layout(
            template="plotly_dark",
            height=300,
            xaxis=dict(visible=False),
            yaxis=dict(visible=False),
            margin=dict(l=20, r=20, t=40, b=20),
            paper_bgcolor='#0a0a0a',
            plot_bgcolor='#0a0a0a'
        )
        return fig
    
    labels = [d.get("label", "Unknown") for d in detections]
    conf = [d.get("confidence", 0) * 100 for d in detections]
    colors = [THREAT_COLORS.get(d.get("threat_level", "NORMAL"), "#6c757d") for d in detections]
    
    fig = go.Figure(go.Bar(
        x=labels,
        y=conf,
        marker_color=colors,
        text=[f"{c:.1f}%" for c in conf],
        textposition='outside',
        hovertemplate='<b>%{x}</b><br>Confidence: %{y:.1f}%<extra></extra>'
    ))
    
    fig.update_layout(
        title="Active Detections",
        xaxis_title="Detection Type",
        yaxis_title="Confidence (%)",
        template="plotly_dark",
        height=300,
        yaxis_range=[0, 110],
        margin=dict(l=50, r=20, t=50, b=50),
        paper_bgcolor='#0a0a0a',
        plot_bgcolor='#0a0a0a'
    )
    
    return fig


# ===== THREAT HISTORY GRAPH 

@app.callback(
    Output("threat-history-graph", "figure"),
    Input("update-interval", "n_intervals"),
)
def update_threat_history(n):
    """Update threat level history chart"""
    
    if not node or not node.threat_history:
        fig = go.Figure()
        fig.add_annotation(
            text=" No threat history yet",
            xref="paper", yref="paper",
            x=0.5, y=0.5,
            showarrow=False,
            font=dict(size=20, color="#888")
        )
        fig.update_layout(
            template="plotly_dark",
            height=300,
            xaxis=dict(visible=False),
            yaxis=dict(visible=False),
            margin=dict(l=20, r=20, t=40, b=20),
            paper_bgcolor='#0a0a0a',
            plot_bgcolor='#0a0a0a'
        )
        return fig
    
    history = node.threat_history[-50:]
    level_map = {'NORMAL': 0, 'MEDIUM': 1, 'HIGH': 2, 'CRITICAL': 3}
    levels = [level_map.get(h['level'], 0) for h in history]
    
    fig = go.Figure(go.Scatter(
        y=levels,
        mode='lines+markers',
        line=dict(color='#17a2b8', width=3),
        marker=dict(size=6, color='#17a2b8'),
        fill='tozeroy',
        fillcolor='rgba(23, 162, 184, 0.3)',
        hovertemplate='Threat Level: %{text}<extra></extra>',
        text=[h['level'] for h in history]
    ))
    
    fig.update_layout(
        title="Threat Trend (Last 50 readings)",
        yaxis=dict(
            tickmode='array',
            tickvals=[0, 1, 2, 3],
            ticktext=['NORMAL', 'MEDIUM', 'HIGH', 'CRITICAL'],
            gridcolor='#444'
        ),
        xaxis=dict(
            title="Time (readings)",
            gridcolor='#444'
        ),
        template="plotly_dark",
        height=300,
        margin=dict(l=80, r=20, t=50, b=50),
        paper_bgcolor='#0a0a0a',
        plot_bgcolor='#0a0a0a'
    )
    
    return fig


# ===== DETECTION COUNT 

@app.callback(
    Output("detection-count", "children"),
    Input("update-interval", "n_intervals"),
)
def update_detection_count(n):
    """Update detection count summary"""
    
    if not node:
        return " Initializing..."
    
    count = len(node.ai_detections)
    level = node.threat_level
    
    if count == 0:
        return html.Span("All Clear - No Threats Detected", style={"color": "#28a745"})
    
    icons = {"NORMAL": "ℹ ", "MEDIUM": " ", "HIGH": " ", "CRITICAL": " "}
    icon = icons.get(level, "ℹ ")
    color = THREAT_COLORS.get(level, "#6c757d")
    
    return html.Span(
        f"{icon} {count} Active Detection(s) - {level} Threat Level",
        style={"color": color}
    )


# ===== FLIGHT CONTROL CALLBACKS 

@app.callback(Output("btn-arm", "n_clicks"), Input("btn-arm", "n_clicks"), prevent_initial_call=True)
def arm(n):
    if node and n: node.send_command({"cmd": "arm", "confirm": True})
    return 0

@app.callback(Output("btn-disarm", "n_clicks"), Input("btn-disarm", "n_clicks"), prevent_initial_call=True)
def disarm(n):
    if node and n: node.send_command({"cmd": "disarm", "confirm": True})
    return 0

@app.callback(Output("btn-takeoff", "n_clicks"), Input("btn-takeoff", "n_clicks"), prevent_initial_call=True)
def takeoff(n):
    if node and n: node.send_command({"cmd": "takeoff", "alt": 10, "confirm": True})
    return 0

@app.callback(Output("btn-land", "n_clicks"), Input("btn-land", "n_clicks"), prevent_initial_call=True)
def land(n):
    if node and n: node.send_command({"cmd": "land", "confirm": True})
    return 0

@app.callback(Output("btn-hold", "n_clicks"), Input("btn-hold", "n_clicks"), prevent_initial_call=True)
def hold(n):
    if node and n: node.send_command({"cmd": "set_mode", "mode": "HOLD", "confirm": True})
    return 0


# Add to dashboard_ui.py callbacks:

@app.callback(
    Output("training-status", "children"),
    Input("update-interval", "n_intervals"),
)
def update_training_status(n):
    """Show self-learning training status"""
    if not node:
        return ""
    
    # Check if training topic exists
    training_data = getattr(node, 'training_status', None)
    
    if training_data:
        status = training_data.get('status', 'idle')
        
        if status == 'training_started':
            return html.Div([
                html.I(className="fas fa-spinner fa-spin me-2"),
                html.Span(" Model retraining in progress...", 
                         style={"color": "#ffc107"})
            ])
        elif status == 'training_complete':
            version = training_data.get('model_version', '?')
            return html.Div([
                html.I(className="fas fa-check-circle me-2"),
                html.Span(f"✓ Model v{version} trained successfully", 
                         style={"color": "#28a745"})
            ])
    
    return html.Span(" Self-learning active", style={"color": "#17a2b8"})




if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("MAYURI SURVEILLANCE COMMAND & CONTROL DASHBOARD")
    print("=" * 80)
    print(f"\n Dashboard URL: http://localhost:{DASHBOARD_PORT}")
    print(f"Network URL: http://0.0.0.0:{DASHBOARD_PORT}")
    print(f"\n Static files: {STATIC_DIR}")
    print(f" Assets folder: {ASSETS_DIR}")
    print("\n ROS2 Topics:")
    print("   Subscribing:")
    print("     • /mayuri/flight/status     (Drone telemetry)")
    print("     • /mayuri/ai/detections     (AI threat data)")
    print("     • /mayuri/failsafe/event    (Emergency events)")
    print("     • /mayuri/camera/stream     (Live camera feed)")
    print("   Publishing:")
    print("     • /mayuri/flight/cmd        (Flight commands)")
    print("\nFeatures:")
    print("   ✓ Real-time drone telemetry")
    print("   ✓ Interactive map visualization")
    print("   ✓ Live camera with AI overlays")
    print("   ✓ Threat level monitoring")
    print("   ✓ Flight control interface")
    print("   ✓ Auto-recovery from errors")
    print("\n" + "=" * 80)
    print("\n⌨ Press Ctrl+C to stop\n")
    
    try:
        app.run(
            host="0.0.0.0",
            port=DASHBOARD_PORT,
            debug=False
        )
    except KeyboardInterrupt:
        print("\n\n Shutting down dashboard...")
        ros_thread_running = False
    except Exception as e:
        print(f"\n Error: {e}")
    finally:
        print("✓ Dashboard stopped\n")
