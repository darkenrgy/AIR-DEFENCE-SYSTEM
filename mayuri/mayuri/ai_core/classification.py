import os
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
from datetime import datetime
import numpy as np
import base64
from pathlib import Path
import pickle
import threading
import time

# PyTorch
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights
from torchvision import transforms

# TensorFlow
import tensorflow as tf
from tensorflow import keras

# YOLO
from ultralytics import YOLO

from PIL import Image
from collections import deque, defaultdict



class SelfLearningDataManager:
    """Manages training data collection and storage"""
    
    def __init__(self, data_dir="mayuri/ai_core/training_data"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # Directories for different classes
        self.class_dirs = {
            'weapon_gun': self.data_dir / "weapons" / "gun",
            'weapon_knife': self.data_dir / "weapons" / "knife",
            'weapon_pistol': self.data_dir / "weapons" / "pistol",
            'weapon_rifle': self.data_dir / "weapons" / "rifle",
            'person': self.data_dir / "persons",
            'activity_fighting': self.data_dir / "activities" / "fighting",
            'activity_running': self.data_dir / "activities" / "running",
            'activity_normal': self.data_dir / "activities" / "normal",
            'negative': self.data_dir / "negative"  # No weapons/threats
        }
        
        for class_dir in self.class_dirs.values():
            class_dir.mkdir(parents=True, exist_ok=True)
        
        # Training metadata
        self.metadata_file = self.data_dir / "training_metadata.json"
        self.load_metadata()
        
        # Statistics
        self.samples_collected = defaultdict(int)
        self.samples_used_for_training = defaultdict(int)
        
        print(f"✓ Self-Learning Data Manager initialized: {self.data_dir}")
    
    
    def load_metadata(self):
        """Load training metadata"""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {
                'total_samples': 0,
                'last_training': None,
                'model_versions': [],
                'class_distribution': {}
            }
    
    
    def save_metadata(self):
        """Save training metadata"""
        with open(self.metadata_file, 'w') as f:
            json.dump(self.metadata, f, indent=2)
    
    
    def save_detection_sample(self, frame, bbox, label, confidence, source="detection"):
        """Save a detection sample for future training"""
        
        # Only save high-confidence detections for self-training
        if confidence < 0.75:
            return False
        
        # Extract ROI from frame
        x1, y1, x2, y2 = map(int, bbox)
        roi = frame[y1:y2, x1:x2]
        
        if roi.size == 0:
            return False
        
        # Determine class directory
        class_key = f"weapon_{label}" if label in ['gun', 'knife', 'pistol', 'rifle'] else label
        if class_key not in self.class_dirs:
            class_key = 'negative'
        
        class_dir = self.class_dirs[class_key]
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{label}_{confidence:.2f}_{timestamp}.jpg"
        filepath = class_dir / filename
        
        # Save image
        cv2.imwrite(str(filepath), roi)
        
        # Save metadata
        meta_filepath = filepath.with_suffix('.json')
        metadata = {
            'label': label,
            'confidence': confidence,
            'bbox': bbox,
            'source': source,
            'timestamp': timestamp,
            'class_key': class_key
        }
        
        with open(meta_filepath, 'w') as f:
            json.dump(metadata, f)
        
        # Update statistics
        self.samples_collected[class_key] += 1
        self.metadata['total_samples'] += 1
        self.metadata['class_distribution'][class_key] = \
            self.metadata['class_distribution'].get(class_key, 0) + 1
        
        if self.metadata['total_samples'] % 100 == 0:
            self.save_metadata()
            print(f" Collected {self.metadata['total_samples']} training samples")
        
        return True
    
    
    def get_training_dataset(self, class_name=None):
        """Get all training samples for a class or all classes"""
        samples = []
        
        if class_name:
            class_dirs = [self.class_dirs[class_name]]
        else:
            class_dirs = self.class_dirs.values()
        
        for class_dir in class_dirs:
            for img_file in class_dir.glob("*.jpg"):
                meta_file = img_file.with_suffix('.json')
                if meta_file.exists():
                    with open(meta_file, 'r') as f:
                        metadata = json.load(f)
                    
                    samples.append({
                        'image_path': str(img_file),
                        'label': metadata['label'],
                        'class_key': metadata['class_key'],
                        'confidence': metadata['confidence']
                    })
        
        return samples
    
    
    def get_sample_counts(self):
        """Get count of samples per class"""
        counts = {}
        for class_key, class_dir in self.class_dirs.items():
            counts[class_key] = len(list(class_dir.glob("*.jpg")))
        return counts



class OnlineLearningWeaponDetector(nn.Module):
    """Weapon detector with online learning capabilities"""
    
    def __init__(self, num_classes=5):
        super().__init__()
        
        # Base model
        self.model = fasterrcnn_resnet50_fpn_v2(
            weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        )
        
        # Modify for our classes: background, gun, pistol, rifle, knife
        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = \
            torchvision.models.detection.faster_rcnn.FastRCNNPredictor(
                in_features, num_classes
            )
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.to(self.device)
        
        # Optimizer for online learning
        self.optimizer = optim.Adam(self.parameters(), lr=0.0001)
        
        # Training buffer (experience replay)
        self.experience_buffer = deque(maxlen=1000)
        
        # Class mapping
        self.class_to_idx = {
            'background': 0,
            'gun': 1,
            'pistol': 2,
            'rifle': 3,
            'knife': 4
        }
        self.idx_to_class = {v: k for k, v in self.class_to_idx.items()}
        
        # Model versioning
        self.model_dir = Path("mayuri/ai_core/models/checkpoints")
        self.model_dir.mkdir(parents=True, exist_ok=True)
        self.current_version = 0
        
        # Load latest model if exists
        self.load_latest_model()
        
        print("✓ Online Learning Weapon Detector initialized")
    
    
    def forward(self, images):
        """Forward pass"""
        return self.model(images)
    
    
    def add_experience(self, image, detections):
        """Add detection to experience buffer for replay"""
        self.experience_buffer.append({
            'image': image,
            'detections': detections,
            'timestamp': time.time()
        })
    
    
    def online_train_step(self, images, targets):
        """Single online training step"""
        self.train()
        
        # Forward pass
        loss_dict = self.model(images, targets)
        losses = sum(loss for loss in loss_dict.values())
        
        # Backward pass
        self.optimizer.zero_grad()
        losses.backward()
        
        # Gradient clipping to prevent instability
        torch.nn.utils.clip_grad_norm_(self.parameters(), max_norm=1.0)
        
        self.optimizer.step()
        
        self.eval()
        
        return losses.item()
    
    
    def incremental_train(self, data_manager, num_epochs=1, batch_size=2):
        """Incremental training from collected data"""
        
        samples = data_manager.get_training_dataset()
        
        if len(samples) < 10:
            print(f" Not enough samples for training ({len(samples)})")
            return False
        
        print(f" Starting incremental training with {len(samples)} samples...")
        
        self.train()
        
        for epoch in range(num_epochs):
            epoch_loss = 0.0
            num_batches = 0
            
            # Shuffle samples
            np.random.shuffle(samples)
            
            for i in range(0, len(samples), batch_size):
                batch_samples = samples[i:i+batch_size]
                
                images = []
                targets = []
                
                for sample in batch_samples:
                    # Load image
                    img = cv2.imread(sample['image_path'])
                    if img is None:
                        continue
                    
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img_tensor = transforms.ToTensor()(img).to(self.device)
                    images.append(img_tensor)
                    
                    # Create target (simplified - bbox would be whole image)
                    h, w = img.shape[:2]
                    label = sample['label']
                    class_idx = self.class_to_idx.get(label, 0)
                    
                    target = {
                        'boxes': torch.tensor([[0, 0, w, h]], dtype=torch.float32).to(self.device),
                        'labels': torch.tensor([class_idx], dtype=torch.int64).to(self.device)
                    }
                    targets.append(target)
                
                if len(images) == 0:
                    continue
                
                # Train step
                loss = self.online_train_step(images, targets)
                epoch_loss += loss
                num_batches += 1
            
            avg_loss = epoch_loss / max(num_batches, 1)
            print(f"   Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.4f}")
        
        self.eval()
        
        # Save new model version
        self.save_model()
        
        # Update metadata
        data_manager.metadata['last_training'] = datetime.now().isoformat()
        data_manager.metadata['model_versions'].append({
            'version': self.current_version,
            'timestamp': datetime.now().isoformat(),
            'samples_used': len(samples)
        })
        data_manager.save_metadata()
        
        print(f"✓ Incremental training complete. Model v{self.current_version} saved.")
        return True
    
    
    def save_model(self):
        """Save model checkpoint"""
        self.current_version += 1
        checkpoint_path = self.model_dir / f"weapon_detector_v{self.current_version}.pth"
        
        torch.save({
            'version': self.current_version,
            'model_state_dict': self.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'timestamp': datetime.now().isoformat()
        }, checkpoint_path)
        
        print(f" Model saved: {checkpoint_path}")
    
    
    def load_latest_model(self):
        """Load the latest model checkpoint"""
        checkpoints = sorted(self.model_dir.glob("weapon_detector_v*.pth"))
        
        if checkpoints:
            latest_checkpoint = checkpoints[-1]
            checkpoint = torch.load(latest_checkpoint, map_location=self.device)
            
            self.load_state_dict(checkpoint['model_state_dict'])
            self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
            self.current_version = checkpoint['version']
            
            print(f"✓ Loaded model v{self.current_version} from {latest_checkpoint}")
            return True
        
        return False
    
    
    @torch.no_grad()
    def detect(self, frame):
        """Detect weapons in frame"""
        self.eval()
        
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_tensor = transforms.ToTensor()(image_rgb).to(self.device)
        
        predictions = self([image_tensor])[0]
        
        detections = []
        boxes = predictions['boxes'].cpu().numpy()
        labels = predictions['labels'].cpu().numpy()
        scores = predictions['scores'].cpu().numpy()
        
        for box, label, score in zip(boxes, labels, scores):
            if score > 0.6 and label > 0:  # Exclude background
                class_name = self.idx_to_class.get(label, 'unknown')
                detections.append({
                    'type': class_name,
                    'confidence': float(score),
                    'bbox': box.tolist(),
                    'model': 'online_learning'
                })
        
        return detections



class SelfImprovingSurveillance(Node):
    def __init__(self):
        super().__init__("self_improving_surveillance")
        
        # Parameters
        self.declare_parameter('camera_source', 0)
        self.declare_parameter('auto_train_interval', 3600)  # Retrain every hour
        self.declare_parameter('min_samples_for_training', 100)
        self.declare_parameter('save_detections', True)
        
        camera_source = self.get_parameter('camera_source').value
        self.auto_train_interval = self.get_parameter('auto_train_interval').value
        self.min_samples = self.get_parameter('min_samples_for_training').value
        self.save_detections = self.get_parameter('save_detections').value
        
        # Initialize camera
        self.get_logger().info(f" Opening camera: {camera_source}")
        self.camera = cv2.VideoCapture(camera_source)
        
        if not self.camera.isOpened():
            self.get_logger().error(" Camera failed!")
            self.camera = None
        else:
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.get_logger().info("✓ Camera ready")
        
        # Initialize data manager
        self.data_manager = SelfLearningDataManager()
        
        # Initialize online learning detector
        self.get_logger().info("Loading online learning weapon detector...")
        self.weapon_detector = OnlineLearningWeaponDetector()
        
        # YOLO for validation
        self.yolo = YOLO("yolov8m.pt")
        
        # Publishers
        self.ai_pub = self.create_publisher(String, "/mayuri/ai/detections", 10)
        self.camera_pub = self.create_publisher(String, "/mayuri/camera/stream", 10)
        self.training_pub = self.create_publisher(String, "/mayuri/training/status", 10)
        
        # State
        self.frame_count = 0
        self.processing = False
        self.last_training_time = time.time()
        
        # Timer for processing
        self.create_timer(0.1, self.process_frame)  # 10 Hz
        
        # Timer for auto-retraining
        self.create_timer(60.0, self.check_auto_retrain)  # Check every minute
        
        self.get_logger().info(" Self-Improving Surveillance System Online")
        self.get_logger().info(f"   Auto-retrain interval: {self.auto_train_interval}s")
        self.get_logger().info(f"   Min samples for training: {self.min_samples}")
    
    
    def capture_frame(self):
        """Capture frame from camera"""
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                return frame
        
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.putText(frame, "NO CAMERA", (400, 360),
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
        return frame
    
    
    def process_frame(self):
        """Main processing with self-learning"""
        if self.processing:
            return
        
        self.processing = True
        self.frame_count += 1
        
        frame = self.capture_frame()
        
        try:
            # Weapon detection
            weapon_detections = self.weapon_detector.detect(frame)
            
            # YOLO validation
            yolo_results = self.yolo(frame, conf=0.5, verbose=False)[0]
            person_count = sum(1 for box in yolo_results.boxes 
                              if yolo_results.names[int(box.cls)] == 'person')
            
            # Save high-confidence detections for self-learning
            if self.save_detections:
                for det in weapon_detections:
                    if det['confidence'] > 0.75:
                        self.data_manager.save_detection_sample(
                            frame, det['bbox'], det['type'], det['confidence']
                        )
                
                # Also save negative samples (no weapons)
                if len(weapon_detections) == 0 and self.frame_count % 100 == 0:
                    h, w = frame.shape[:2]
                    self.data_manager.save_detection_sample(
                        frame, [0, 0, w, h], 'negative', 1.0
                    )
            
            # Threat analysis
            threats = self.analyze_threats(weapon_detections, person_count)
            
            # Draw and publish
            annotated_frame = self.draw_detections(frame, weapon_detections, yolo_results, threats)
            dashboard_data = self.format_for_dashboard(threats)
            self.publish_to_dashboard(dashboard_data, annotated_frame)
        
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
        
        finally:
            self.processing = False
    
    
    def check_auto_retrain(self):
        """Check if it's time to retrain"""
        current_time = time.time()
        time_since_training = current_time - self.last_training_time
        
        if time_since_training < self.auto_train_interval:
            return
        
        # Check if we have enough samples
        sample_counts = self.data_manager.get_sample_counts()
        total_samples = sum(sample_counts.values())
        
        if total_samples < self.min_samples:
            self.get_logger().info(
                f" Samples collected: {total_samples}/{self.min_samples} "
                f"(need more before retraining)"
            )
            return
        
        # Start retraining in background thread
        self.get_logger().info(f" Starting auto-retraining with {total_samples} samples...")
        
        training_thread = threading.Thread(target=self.retrain_models, daemon=True)
        training_thread.start()
        
        self.last_training_time = current_time
    
    
    def retrain_models(self):
        """Retrain models with collected data"""
        try:
            # Publish training status
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'training_started',
                'timestamp': datetime.now().isoformat()
            })
            self.training_pub.publish(status_msg)
            
            # Incremental training
            success = self.weapon_detector.incremental_train(
                self.data_manager,
                num_epochs=3,
                batch_size=4
            )
            
            if success:
                self.get_logger().info("✓ Model retraining successful!")
                
                status_msg.data = json.dumps({
                    'status': 'training_complete',
                    'timestamp': datetime.now().isoformat(),
                    'model_version': self.weapon_detector.current_version
                })
            else:
                status_msg.data = json.dumps({
                    'status': 'training_failed',
                    'timestamp': datetime.now().isoformat()
                })
            
            self.training_pub.publish(status_msg)
        
        except Exception as e:
            self.get_logger().error(f"Training error: {e}")
    
    
    def analyze_threats(self, weapon_detections, person_count):
        """Analyze threats"""
        threats = {
            'timestamp': datetime.now().isoformat(),
            'armed_persons': len(weapon_detections),
            'weapons_detected': weapon_detections,
            'person_count': person_count,
            'threat_level': 'NORMAL',
            'confidence': 0.0
        }
        
        threat_score = 0.0
        
        if len(weapon_detections) > 0:
            weapon_conf = np.mean([d['confidence'] for d in weapon_detections])
            threat_score += weapon_conf * 50
        
        if person_count > 10:
            threat_score += 20
        
        if threat_score > 60:
            threats['threat_level'] = 'CRITICAL'
        elif threat_score > 40:
            threats['threat_level'] = 'HIGH'
        elif threat_score > 20:
            threats['threat_level'] = 'MEDIUM'
        
        threats['confidence'] = min(threat_score / 100.0, 1.0)
        
        return threats
    
    
    def draw_detections(self, frame, weapon_detections, yolo_results, threats):
        """Draw detections"""
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        
        # Weapons
        for weapon in weapon_detections:
            bbox = weapon.get('bbox')
            if bbox:
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
                label = f" {weapon['type']} {weapon['confidence']*100:.1f}%"
                cv2.putText(annotated, label, (x1, y1-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Persons
        for box in yolo_results.boxes:
            if yolo_results.names[int(box.cls)] == 'person':
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # HUD
        cv2.rectangle(annotated, (0, 0), (w, 100), (0, 0, 0), -1)
        cv2.putText(annotated, f"THREAT: {threats['threat_level']}", (10, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255) if threats['threat_level'] == 'CRITICAL' else (0, 255, 0), 2)
        
        # Show model version and training stats
        model_info = f"Model v{self.weapon_detector.current_version}"
        cv2.putText(annotated, model_info, (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        sample_counts = self.data_manager.get_sample_counts()
        total_samples = sum(sample_counts.values())
        samples_info = f"Training samples: {total_samples}"
        cv2.putText(annotated, samples_info, (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return annotated
    
    
    def format_for_dashboard(self, threats):
        """Format for dashboard"""
        detections = []
        
        for weapon in threats['weapons_detected']:
            detections.append({
                'label': f" {weapon['type'].upper()}",
                'confidence': weapon['confidence'],
                'type': 'weapon',
                'threat_level': 'CRITICAL'
            })
        
        if threats['person_count'] > 0:
            detections.append({
                'label': f"👥 {threats['person_count']} Persons",
                'confidence': 1.0,
                'type': 'crowd',
                'threat_level': 'MEDIUM'
            })
        
        return {
            'detections': detections,
            'overall_threat': threats['threat_level'],
            'timestamp': threats['timestamp']
        }
    
    
    def publish_to_dashboard(self, dashboard_data, annotated_frame):
        """Publish to dashboard"""
        # Detection data
        msg = String()
        msg.data = json.dumps(dashboard_data)
        self.ai_pub.publish(msg)
        
        # Camera stream
        _, buffer = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        camera_msg = String()
        camera_msg.data = json.dumps({
            'image': img_base64,
            'timestamp': dashboard_data['timestamp'],
            'threat_level': dashboard_data['overall_threat']
        })
        self.camera_pub.publish(camera_msg)
    
    
    def destroy_node(self):
        if self.camera:
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SelfImprovingSurveillance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
