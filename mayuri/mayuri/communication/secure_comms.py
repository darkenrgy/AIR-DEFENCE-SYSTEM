import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
import traceback

# Local imports: ensure PYTHONPATH includes project root or package properly installed
try:
    from encryption_manager import EncryptionManager
    from mesh_network import MeshNode
except Exception as e:
    raise RuntimeError("Failed to import local modules (encryption_manager, mesh_network). "
                       "Make sure this file runs from project root or package is installed.") from e


class SecureCommsNode(Node):
    def __init__(self):
        super().__init__('secure_comms')

        # Parameters
        self.declare_parameter('node_name', 'mayuri_node')
        self.declare_parameter('key_dir', 'config/security/keys')
        self.declare_parameter('key_id', '')  # optional
        self.declare_parameter('publish_peer_interval', 5.0)

        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.key_dir = self.get_parameter('key_dir').get_parameter_value().string_value
        self.key_id = self.get_parameter('key_id').get_parameter_value().string_value or None
        self.peer_pub_interval = float(self.get_parameter('publish_peer_interval').get_parameter_value().double_value or 5.0)

        # Encryption manager & Mesh node
        self.get_logger().info(f"Initializing EncryptionManager with key_dir={self.key_dir}")
        self.enc_mgr = EncryptionManager(self.key_dir)

        # Determine key_id: use provided, else use first existing or generate new
        if not self.key_id:
            keys = self.enc_mgr.list_keys()
            if keys:
                self.key_id = list(keys.keys())[0]
                self.get_logger().info(f"No key_id provided — using existing key: {self.key_id}")
            else:
                self.key_id = self.enc_mgr.generate_and_save_key(meta={"purpose": "mayuri_mesh"})
                self.get_logger().info(f"No keys found — generated new key: {self.key_id}")

        # Start underlying mesh
        self.mesh = MeshNode(name=self.node_name, key_dir=self.key_dir, key_id=self.key_id)
        self.mesh.start()

        # ROS publishers & subscribers
        self.outbound_sub = self.create_subscription(String, '/mayuri/outbound_msg', self.on_outbound, 10)
        self.inbound_pub = self.create_publisher(String, '/mayuri/inbound_msg', 10)
        self.peers_pub = self.create_publisher(String, '/mayuri/peers', 1)

        # Thread: monitor incoming queue and publish to ROS
        self._stop_event = threading.Event()
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        # Thread: periodically publish peer list
        self.peer_thread = threading.Thread(target=self._peer_publisher_loop, daemon=True)
        self.peer_thread.start()

        self.get_logger().info("SecureCommsNode initialized and running.")

    # Incoming ROS -> send to mesh peers
    def on_outbound(self, msg: String):
       
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Invalid JSON on /mayuri/outbound_msg: {e}")
            return

        to = payload.get('to', 'all')
        body = payload.get('body', {})
        meta = {k: v for k, v in payload.items() if k not in ['to', 'body']}

        message = {
            'meta': meta,
            'body': body,
            'source': self.node_name,
            'timestamp': time.time()
        }

        # Support direct send to IP or broadcast to known peers
        if to == 'all':
            self.mesh.broadcast(message)
            self.get_logger().debug(f"Broadcasted message to peers: {message.get('type', '')}")
        else:
            # If 'to' looks like IP address, send to that IP; else try match by node name
            target_ip = None
            if isinstance(to, str):
                # simple check for dotted decimal
                if to.count('.') == 3:
                    target_ip = to
                else:
                    # find peer by name
                    for nid, info in list(self.mesh.peers.items()):
                        if info.get('name') == to:
                            target_ip = info.get('ip')
                            break
            if target_ip:
                self.mesh.send(target_ip, message)
                self.get_logger().debug(f"Sent to {target_ip}: {message.get('type','')}")
            else:
                self.get_logger().warning(f"Target not found for 'to': {to}")

    # Worker loop: read mesh.incoming queue and publish to ROS topic
    def _worker_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                # mesh.incoming contains {from: name, message: dict}
                while not self.mesh.incoming.empty():
                    item = self.mesh.incoming.get_nowait()
                    # Wrap as JSON string and publish
                    out_msg = String()
                    # Attach sender IP if available (mesh stores in peers table)
                    self.get_logger().info(f"Incoming mesh msg from {item.get('from')}: {item.get('message')}")
                    out_msg.data = json.dumps({
                        'from': item.get('from'),
                        'message': item.get('message')
                    })
                    self.inbound_pub.publish(out_msg)
            except Exception:
                self.get_logger().warning('Exception in worker_loop:\n' + traceback.format_exc())
            time.sleep(0.05)

    def _peer_publisher_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                # Build peer list
                peers_snapshot = {}
                for nid, info in self.mesh.peers.items():
                    peers_snapshot[nid] = {
                        'name': info.get('name'),
                        'ip': info.get('ip'),
                        'last_seen': info.get('last_seen')
                    }
                msg = String()
                msg.data = json.dumps({
                    'node_id': self.mesh.node_id,
                    'peers': peers_snapshot,
                    'timestamp': time.time()
                })
                self.peers_pub.publish(msg)
            except Exception:
                self.get_logger().warning('Exception in peer_publisher_loop:\n' + traceback.format_exc())
            time.sleep(self.peer_pub_interval)

    def destroy_node(self):
        self.get_logger().info("Shutting down SecureCommsNode...")
        self._stop_event.set()
        try:
            if self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.peer_thread.is_alive():
                self.peer_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.mesh.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SecureCommsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
