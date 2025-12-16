import socket
import threading
import json
import time
import uuid
import queue
from typing import Dict, Any, Optional

from encryption_manager import EncryptionManager

# --- Default Configuration ---
BROADCAST_PORT = 50555
UNICAST_PORT = 50556
BROADCAST_INTERVAL = 3.0  # seconds
BUFFER_SIZE = 65535


class MeshNode:
    def __init__(self, name: str, key_dir: str = "config/security/keys", key_id: Optional[str] = None):
        self.name = name
        self.node_id = uuid.uuid4().hex[:8]
        self.running = False
        self.peers: Dict[str, Dict[str, Any]] = {}
        self.incoming = queue.Queue()

        # Encryption
        self.enc_mgr = EncryptionManager(key_dir)
        if key_id is None:
            # use first key if exists, else generate
            keys = self.enc_mgr.list_keys()
            if not keys:
                key_id = self.enc_mgr.generate_and_save_key(meta={"purpose": "mesh_comm"})
            else:
                key_id = list(keys.keys())[0]
        self.key_id = key_id

        # UDP sockets
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.unicast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.unicast_sock.bind(("", UNICAST_PORT))

        # Threads
        self._threads = []

    # --------------------------------------------------------------------
    def start(self):
        self.running = True
        self._threads = [
            threading.Thread(target=self._broadcast_loop, daemon=True),
            threading.Thread(target=self._receive_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()
        print(f" [{self.name}] MeshNode started (ID={self.node_id}) on UDP {UNICAST_PORT}")

    def stop(self):
        self.running = False
        for t in self._threads:
            if t.is_alive():
                t.join(timeout=0.5)
        self.broadcast_sock.close()
        self.unicast_sock.close()
        print(f" [{self.name}] MeshNode stopped")

    # --------------------------------------------------------------------
    def _broadcast_loop(self):
        """Broadcast 'hello' message periodically for peer discovery."""
        msg = {
            "type": "HELLO",
            "node_id": self.node_id,
            "name": self.name,
            "timestamp": time.time(),
        }
        while self.running:
            try:
                data = json.dumps(msg).encode("utf-8")
                self.broadcast_sock.sendto(data, ("<broadcast>", BROADCAST_PORT))
            except Exception as e:
                print(f"[{self.name}] Broadcast error: {e}")
            time.sleep(BROADCAST_INTERVAL)

    # --------------------------------------------------------------------
    def _receive_loop(self):
        """Listen for unicast + broadcast messages."""
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_sock.bind(("", BROADCAST_PORT))

        while self.running:
            try:
                readable_socks = [recv_sock, self.unicast_sock]
                for s in readable_socks:
                    s.settimeout(0.2)
                    try:
                        data, addr = s.recvfrom(BUFFER_SIZE)
                        self._handle_packet(data, addr)
                    except socket.timeout:
                        continue
            except Exception as e:
                print(f"[{self.name}] Receive loop error: {e}")

    # --------------------------------------------------------------------
    def _handle_packet(self, data: bytes, addr):
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception:
            return  # ignore invalid JSON

        msg_type = msg.get("type")

        if msg_type == "HELLO":
            nid = msg.get("node_id")
            if nid and nid != self.node_id:
                if nid not in self.peers:
                    print(f" [{self.name}] Discovered peer: {msg.get('name')} ({addr[0]})")
                self.peers[nid] = {
                    "name": msg.get("name"),
                    "ip": addr[0],
                    "last_seen": time.time()
                }
        elif msg_type == "DATA":
            # Decrypt if encrypted
            payload = msg.get("payload")
            if payload:
                try:
                    pt = self.enc_mgr.decrypt(payload, key_id=self.key_id)
                    message = json.loads(pt.decode("utf-8"))
                    self.incoming.put({"from": msg.get("from"), "message": message})
                    print(f" [{self.name}] Message received from {msg.get('from')}: {message}")
                except Exception as e:
                    print(f"[{self.name}] Failed to decrypt message: {e}")
        else:
            pass  # unknown message type

    # --------------------------------------------------------------------
    def send(self, peer_ip: str, message: Dict[str, Any]):
        """Send an encrypted JSON message to a peer."""
        try:
            pt = json.dumps(message).encode("utf-8")
            payload = self.enc_mgr.encrypt(pt, key_id=self.key_id)
            pkt = {
                "type": "DATA",
                "from": self.name,
                "payload": payload
            }
            self.unicast_sock.sendto(json.dumps(pkt).encode("utf-8"), (peer_ip, UNICAST_PORT))
            print(f" [{self.name}] Sent encrypted message to {peer_ip}")
        except Exception as e:
            print(f"[{self.name}] Send error: {e}")

    # --------------------------------------------------------------------
    def broadcast(self, message: Dict[str, Any]):
        """Send an encrypted message to all known peers."""
        for nid, pinfo in self.peers.items():
            self.send(pinfo["ip"], message)


# --------------------------------------------------------------------------
# Example usage
def _demo():
    node = MeshNode("NodeA")
    node.start()

    try:
        while True:
            # after discovery, send a test message every few seconds
            time.sleep(5)
            if node.peers:
                for nid, peer in node.peers.items():
                    node.send(peer["ip"], {"status": "hello from NodeA"})
            # read incoming messages
            while not node.incoming.empty():
                msg = node.incoming.get()
                print(f"Received from {msg['from']}: {msg['message']}")
    except KeyboardInterrupt:
        node.stop()


if __name__ == "__main__":
    _demo()
