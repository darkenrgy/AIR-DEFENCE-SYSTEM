import json
import socket
import threading
import queue
import time
from typing import Dict, Any, Optional

from encryption_manager import EncryptionManager


class ThreadBroadcaster:
    

    def __init__(self,
                 enc_mgr: Optional[EncryptionManager] = None,
                 key_id: Optional[str] = None,
                 port: int = 50556,
                 num_threads: int = 2,
                 send_rate_limit: float = 0.01):
       
        self.enc_mgr = enc_mgr
        self.key_id = key_id
        self.port = port
        self.send_rate_limit = send_rate_limit
        self.peers = set()  # IP addresses
        self.queue = queue.Queue()
        self.running = False
        self.threads = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Start worker threads
        self.running = True
        for i in range(num_threads):
            t = threading.Thread(target=self._worker_loop, name=f"Broadcaster-{i}", daemon=True)
            self.threads.append(t)
            t.start()

    # -------------------------------------------------------------------------
    def add_peer(self, ip: str):
        """Add a peer IP to broadcast targets."""
        self.peers.add(ip)

    def remove_peer(self, ip: str):
        """Remove a peer from the target list."""
        if ip in self.peers:
            self.peers.remove(ip)

    def broadcast(self, message: Dict[str, Any]):
        """Send message to all peers asynchronously."""
        for ip in self.peers:
            self.queue.put((ip, message))

    def send(self, ip: str, message: Dict[str, Any]):
        """Send message to a single peer asynchronously."""
        self.queue.put((ip, message))

    # -------------------------------------------------------------------------
    def _worker_loop(self):
        """Worker threads continuously take tasks from the queue and send."""
        while self.running:
            try:
                ip, message = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                # Serialize message
                data = json.dumps(message).encode("utf-8")

                # Encrypt if encryption manager provided
                if self.enc_mgr and self.key_id:
                    try:
                        payload = self.enc_mgr.encrypt(data, key_id=self.key_id)
                        data = payload.encode("utf-8")
                    except Exception as e:
                        print(f"[ThreadBroadcaster] Encryption error: {e}")
                        continue

                # Send over UDP
                self.sock.sendto(data, (ip, self.port))
                time.sleep(self.send_rate_limit)

            except Exception as e:
                print(f"[ThreadBroadcaster] Send error to {ip}: {e}")

    # -------------------------------------------------------------------------
    def stop(self):
        """Gracefully stop broadcaster and worker threads."""
        self.running = False
        # Clear queue
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break
        for t in self.threads:
            if t.is_alive():
                t.join(timeout=1.0)
        self.sock.close()
        print(" ThreadBroadcaster stopped")


# -------------------------------------------------------------------------
# Example usage for standalone testing
def _demo():
    from encryption_manager import EncryptionManager

    enc_mgr = EncryptionManager("config/security/keys_demo")
    # Ensure a key exists
    keys = enc_mgr.list_keys()
    if not keys:
        key_id = enc_mgr.generate_and_save_key(meta={"purpose": "bcast"})
    else:
        key_id = list(keys.keys())[0]

    bc = ThreadBroadcaster(enc_mgr=enc_mgr, key_id=key_id, num_threads=3)
    bc.add_peer("127.0.0.1")  # loopback for test

    # Send a few demo messages
    for i in range(5):
        msg = {"seq": i, "timestamp": time.time(), "data": "Hello from MAYURI"}
        bc.broadcast(msg)
        print(f" Sent message #{i}")
        time.sleep(0.5)

    bc.stop()


if __name__ == "__main__":
    _demo()
