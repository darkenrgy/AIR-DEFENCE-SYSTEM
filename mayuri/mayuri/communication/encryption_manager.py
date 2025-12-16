import os
import json
import base64
import datetime
import uuid
from typing import Optional, Dict

from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding, rsa
from cryptography.hazmat.backends import default_backend


# Helper utilities -----------------------------------------------------------

def b64encode(b: bytes) -> str:
    return base64.b64encode(b).decode('utf-8')

def b64decode(s: str) -> bytes:
    return base64.b64decode(s.encode('utf-8'))

def now_iso() -> str:
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + 'Z'


# Encryption manager ---------------------------------------------------------

class EncryptionManager:
    """
    Manages symmetric AES-256-GCM keys and provides encrypt/decrypt operations.
    Keys are stored on disk in JSON files under key_dir with metadata.

    Optional RSA wrapping/unwrapping functions allow secure transfer of symmetric keys.
    """

    KEY_FILENAME_TEMPLATE = "aeskey_{key_id}.json"
    KEY_FILE_MODE = 0o600

    def __init__(self, key_dir: str = "config/security/keys"):
        self.key_dir = key_dir
        os.makedirs(self.key_dir, exist_ok=True)

    # ----------------- Key generation / storage -----------------

    def generate_key(self) -> bytes:
        """Generate a fresh 32-byte AES-256 key."""
        return AESGCM.generate_key(bit_length=256)

    def _key_filepath(self, key_id: str) -> str:
        return os.path.join(self.key_dir, self.KEY_FILENAME_TEMPLATE.format(key_id=key_id))

    def save_key(self, key: bytes, key_id: Optional[str] = None, meta: Optional[Dict] = None) -> str:
        """
        Save a symmetric key to disk with metadata. Returns the key_id used.
        Key is stored base64-encoded inside a JSON file, file permission set to 600.
        """
        if key_id is None:
            key_id = uuid.uuid4().hex[:16]
        meta = meta or {}
        data = {
            "key_id": key_id,
            "created_at": now_iso(),
            "key_b64": b64encode(key),
            "meta": meta
        }
        path = self._key_filepath(key_id)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        os.chmod(path, self.KEY_FILE_MODE)
        return key_id

    def load_key(self, key_id: str) -> Optional[bytes]:
        """Load a symmetric key from disk by key_id. Returns raw key bytes or None."""
        path = self._key_filepath(key_id)
        if not os.path.isfile(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        key_b64 = data.get("key_b64")
        if not key_b64:
            return None
        return b64decode(key_b64)

    def list_keys(self) -> Dict[str, Dict]:
        """Return a dict of key_id -> metadata for all keys in key_dir."""
        result = {}
        for fname in os.listdir(self.key_dir):
            if not fname.startswith("aeskey_") or not fname.endswith(".json"):
                continue
            path = os.path.join(self.key_dir, fname)
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                key_id = data.get("key_id")
                result[key_id] = {
                    "created_at": data.get("created_at"),
                    "meta": data.get("meta", {})
                }
            except Exception:
                continue
        return result

    def generate_and_save_key(self, key_id: Optional[str] = None, meta: Optional[Dict] = None) -> str:
        k = self.generate_key()
        return self.save_key(k, key_id=key_id, meta=meta)

    def rotate_key(self, old_key_id: str, new_key_id: Optional[str] = None) -> str:
        """
        Generate a new key and save it. Caller is responsible for re-encrypting any stored data
        encrypted with the old key (or use a key-wrapping strategy).
        Returns new_key_id.
        """
        new_key_id = self.generate_and_save_key(key_id=new_key_id)
        # Optionally keep track of rotation mapping in metadata - not automated here
        return new_key_id

    # ----------------- AES-GCM encrypt / decrypt -----------------

    def encrypt(self, plaintext: bytes, key_id: str, aad: Optional[bytes] = None) -> str:
        """
        Encrypt plaintext with AES-256-GCM using key identified by key_id.
        Returns a base64-encoded payload containing: key_id || nonce || ciphertext || tag as JSON string.
        """
        key = self.load_key(key_id)
        if key is None:
            raise ValueError(f"Key {key_id} not found")

        aesgcm = AESGCM(key)
        nonce = os.urandom(12)  # 96-bit nonce recommended for GCM
        aad = aad or b""
        ct = aesgcm.encrypt(nonce, plaintext, aad)  # ct contains ciphertext || tag

        payload = {
            "key_id": key_id,
            "nonce_b64": b64encode(nonce),
            "ct_b64": b64encode(ct),
            "aad_b64": b64encode(aad)
        }
        return json.dumps(payload)

    def decrypt(self, payload_json: str, key_id: Optional[str] = None, aad: Optional[bytes] = None) -> bytes:
        """
        Decrypt a payload produced by encrypt(). If key_id provided, it will be used; otherwise
        key_id is read from payload_json.
        """
        data = json.loads(payload_json)
        key_id = key_id or data.get("key_id")
        if key_id is None:
            raise ValueError("No key_id provided in payload or arguments")

        key = self.load_key(key_id)
        if key is None:
            raise ValueError(f"Key {key_id} not found")

        nonce = b64decode(data["nonce_b64"])
        ct = b64decode(data["ct_b64"])
        payload_aad = b64decode(data.get("aad_b64", "")) if data.get("aad_b64") else b""
        aad_final = aad if aad is not None else payload_aad

        aesgcm = AESGCM(key)
        plaintext = aesgcm.decrypt(nonce, ct, aad_final)
        return plaintext

    # ----------------- Optional: RSA wrap/unwrap symmetric key -----------------

    @staticmethod
    def generate_rsa_keypair(key_size: int = 2048):
        """Return (private_key_obj, public_key_obj)"""
        private_key = rsa.generate_private_key(public_exponent=65537, key_size=key_size, backend=default_backend())
        public_key = private_key.public_key()
        return private_key, public_key

    @staticmethod
    def rsa_serialize_public(public_key) -> bytes:
        return public_key.public_bytes(encoding=serialization.Encoding.PEM,
                                       format=serialization.PublicFormat.SubjectPublicKeyInfo)

    @staticmethod
    def rsa_serialize_private(private_key, password: Optional[bytes] = None) -> bytes:
        encryption = serialization.BestAvailableEncryption(password) if password else serialization.NoEncryption()
        return private_key.private_bytes(encoding=serialization.Encoding.PEM,
                                         format=serialization.PrivateFormat.PKCS8,
                                         encryption_algorithm=encryption)

    @staticmethod
    def rsa_load_public(pem_data: bytes):
        return serialization.load_pem_public_key(pem_data, backend=default_backend())

    @staticmethod
    def rsa_load_private(pem_data: bytes, password: Optional[bytes] = None):
        return serialization.load_pem_private_key(pem_data, password=password, backend=default_backend())

    @staticmethod
    def rsa_wrap_key(symmetric_key: bytes, recipient_public_key_pem: bytes) -> str:
        """
        Wrap (encrypt) a symmetric key using recipient's RSA public key.
        Returns base64 encoded ciphertext (bytes).
        """
        pub = EncryptionManager.rsa_load_public(recipient_public_key_pem)
        ct = pub.encrypt(
            symmetric_key,
            padding.OAEP(mgf=padding.MGF1(algorithm=hashes.SHA256()), algorithm=hashes.SHA256(), label=None)
        )
        return b64encode(ct)

    @staticmethod
    def rsa_unwrap_key(wrapped_b64: str, recipient_private_key_pem: bytes, password: Optional[bytes] = None) -> bytes:
        """
        Unwrap a symmetric key using recipient's RSA private key PEM (bytes). Returns raw symmetric key.
        """
        priv = EncryptionManager.rsa_load_private(recipient_private_key_pem, password=password)
        ct = b64decode(wrapped_b64)
        key = priv.decrypt(ct, padding.OAEP(mgf=padding.MGF1(algorithm=hashes.SHA256()), algorithm=hashes.SHA256(), label=None))
        return key


# ----------------- Quick CLI / example usage -----------------
def _example_run():
    print("EncryptionManager example run\n")
    mgr = EncryptionManager(key_dir="config/security/keys_demo")

    # 1) Create key if none exists
    keys = mgr.list_keys()
    if not keys:
        kid = mgr.generate_and_save_key(meta={"purpose": "demo"})
        print("Generated key:", kid)
    else:
        kid = list(keys.keys())[0]
        print("Using existing key:", kid)

    pt = b"Hello MAYURI - secure message"
    aad = b"mayuri-aad-v1"

    print("Plaintext:", pt)
    payload = mgr.encrypt(pt, key_id=kid, aad=aad)
    print("Encrypted payload (json):", payload)

    dt = mgr.decrypt(payload)
    print("Decrypted:", dt)

    # RSA wrap demo
    priv, pub = mgr.generate_rsa_keypair()
    pub_pem = mgr.rsa_serialize_public(pub)
    priv_pem = mgr.rsa_serialize_private(priv)
    symmetric_key = mgr.load_key(kid)
    wrapped = mgr.rsa_wrap_key(symmetric_key, pub_pem)
    unwrapped = mgr.rsa_unwrap_key(wrapped, priv_pem)
    assert unwrapped == symmetric_key
    print("RSA wrap/unwrap OK")

if __name__ == "__main__":
    _example_run()
