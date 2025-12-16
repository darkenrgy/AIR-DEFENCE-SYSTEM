import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import os
from datetime import datetime
from pathlib import Path

# Audio libraries
try:
    from playsound import playsound
    PLAYSOUND_AVAILABLE = True
except ImportError:
    PLAYSOUND_AVAILABLE = False
    print("playsound not available. Install: pip install playsound")

try:
    import pyttsx3
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False
    print(" pyttsx3 not available. Install: pip install pyttsx3")

# Pygame for beep generation
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class AlertSystem(Node):
    def __init__(self):
        super().__init__("alert_system")
        
        # Parameters
        self.declare_parameter("enable_sound", True)
        self.declare_parameter("enable_tts", True)
        self.declare_parameter("log_path", "mayuri/logs/alerts.log")
        self.declare_parameter("sound_critical", "mayuri/ui_control/assets/sounds/critical.wav")
        self.declare_parameter("sound_high", "mayuri/ui_control/assets/sounds/high.wav")
        self.declare_parameter("sound_warning", "mayuri/ui_control/assets/sounds/warning.wav")
        self.declare_parameter("alert_cooldown", 3.0)  # seconds between same alerts
        
        self.enable_sound = bool(self.get_parameter("enable_sound").value)
        self.enable_tts = bool(self.get_parameter("enable_tts").value)
        self.log_path = str(self.get_parameter("log_path").value)
        self.alert_cooldown = float(self.get_parameter("alert_cooldown").value)
        
        self.sound_files = {
            "CRITICAL": str(self.get_parameter("sound_critical").value),
            "HIGH": str(self.get_parameter("sound_high").value),
            "WARNING": str(self.get_parameter("sound_warning").value),
        }
        
        # Create directories
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        os.makedirs("mayuri/ui_control/assets/sounds", exist_ok=True)
        
        # Initialize TTS engine
        if self.enable_tts and TTS_AVAILABLE:
            try:
                self.tts_engine = pyttsx3.init()
                self.tts_engine.setProperty('rate', 150)  # Speed
                self.tts_engine.setProperty('volume', 1.0)  # Volume
                self.get_logger().info("✓ Text-to-Speech initialized")
            except Exception as e:
                self.get_logger().warn(f"TTS initialization failed: {e}")
                self.tts_engine = None
        else:
            self.tts_engine = None
        
        # Initialize Pygame for beep sounds
        if PYGAME_AVAILABLE:
            try:
                pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
                self.get_logger().info("✓ Pygame audio initialized")
            except Exception as e:
                self.get_logger().warn(f"Pygame init failed: {e}")
        
        # Alert tracking
        self.alert_lock = threading.Lock()
        self.last_alert_time = {}
        self.alert_count = {"CRITICAL": 0, "HIGH": 0, "MEDIUM": 0, "NORMAL": 0}
        
        # Threat tracking
        self.current_threat_level = "NORMAL"
        self.armed_person_detected = False
        
        # Subscriptions
        self.create_subscription(String, "/mayuri/ai/detections", self.on_ai_detections, 10)
        self.create_subscription(String, "/mayuri/failsafe/event", self.on_failsafe_event, 10)
        self.create_subscription(String, "/mayuri/flight/status", self.on_flight_status, 10)
        
        # Create default sound files if they don't exist
        self.create_default_sounds()
        
        self.get_logger().info(" AlertSystem initialized and monitoring...")
        self.get_logger().info(f"   Sound alerts: {'✓ Enabled' if self.enable_sound else '✗ Disabled'}")
        self.get_logger().info(f"   TTS alerts: {'✓ Enabled' if self.enable_tts and self.tts_engine else '✗ Disabled'}")
    
    
    # SOUND GENERATION
    
    
    def create_default_sounds(self):
        """Create default beep sounds if files don't exist"""
        if not PYGAME_AVAILABLE:
            return
        
        for level, filepath in self.sound_files.items():
            if not os.path.exists(filepath):
                try:
                    self.generate_beep_sound(filepath, level)
                    self.get_logger().info(f"Generated default sound: {filepath}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to generate sound {filepath}: {e}")
    
    
    def generate_beep_sound(self, filepath, level):
        """Generate beep sound using sine wave"""
        if not PYGAME_AVAILABLE:
            return
        
        import numpy as np
        from scipy.io import wavfile
        
        # Sound parameters based on threat level
        params = {
            "CRITICAL": {"freq": 1000, "duration": 0.3, "repeats": 3},
            "HIGH": {"freq": 800, "duration": 0.2, "repeats": 2},
            "WARNING": {"freq": 600, "duration": 0.15, "repeats": 1}
        }
        
        config = params.get(level, params["WARNING"])
        
        sample_rate = 22050
        t = np.linspace(0, config["duration"], int(sample_rate * config["duration"]))
        
        # Generate sine wave
        frequency = config["freq"]
        wave = np.sin(2 * np.pi * frequency * t)
        
        # Add fade in/out to avoid clicks
        fade_length = int(sample_rate * 0.01)
        wave[:fade_length] *= np.linspace(0, 1, fade_length)
        wave[-fade_length:] *= np.linspace(1, 0, fade_length)
        
        # Repeat beeps with gaps
        silence = np.zeros(int(sample_rate * 0.1))
        full_wave = wave
        for _ in range(config["repeats"] - 1):
            full_wave = np.concatenate([full_wave, silence, wave])
        
        # Convert to 16-bit PCM
        audio = (full_wave * 32767).astype(np.int16)
        
        # Save as WAV
        wavfile.write(filepath, sample_rate, audio)
    
    
    # ALERT METHODS
   
    
    def play_sound(self, level: str):
        """Play alert sound depending on severity"""
        if not self.enable_sound:
            return
        
        filepath = self.sound_files.get(level)
        if not filepath or not os.path.exists(filepath):
            # Fallback to beep
            self.play_beep(level)
            return
        
        # Play sound in background thread
        if PLAYSOUND_AVAILABLE:
            threading.Thread(target=self._play_sound_thread, args=(filepath,), daemon=True).start()
        elif PYGAME_AVAILABLE:
            try:
                pygame.mixer.music.load(filepath)
                pygame.mixer.music.play()
            except Exception as e:
                self.get_logger().warn(f"Sound playback failed: {e}")
    
    
    def _play_sound_thread(self, filepath):
        """Thread-safe sound playback"""
        try:
            playsound(filepath)
        except Exception as e:
            self.get_logger().warn(f"Playsound error: {e}")
    
    
    def play_beep(self, level: str):
        """Play simple beep if no sound file available"""
        if not PYGAME_AVAILABLE:
            # Terminal beep as last resort
            print('\a', end='', flush=True)
            return
        
        freq_map = {"CRITICAL": 1000, "HIGH": 800, "WARNING": 600}
        frequency = freq_map.get(level, 600)
        
        try:
            # Generate tone
            sample_rate = 22050
            duration = 0.2
            t = np.linspace(0, duration, int(sample_rate * duration))
            wave = np.sin(2 * np.pi * frequency * t)
            audio = (wave * 32767).astype(np.int16)
            
            # Play
            sound = pygame.sndarray.make_sound(audio)
            sound.play()
        except Exception as e:
            print('\a', end='', flush=True)  # Fallback to system beep
    
    
    def speak_alert(self, message: str):
        """Text-to-speech announcement"""
        if not self.enable_tts or not self.tts_engine:
            return
        
        def _speak():
            try:
                self.tts_engine.say(message)
                self.tts_engine.runAndWait()
            except Exception as e:
                self.get_logger().warn(f"TTS error: {e}")
        
        threading.Thread(target=_speak, daemon=True).start()
    
    
    def log_alert(self, level: str, source: str, message: str):
        """Log alert to file with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_line = f"[{timestamp}] [{level}] ({source}) {message}\n"
        
        with self.alert_lock:
            try:
                with open(self.log_path, "a") as f:
                    f.write(log_line)
            except Exception as e:
                self.get_logger().error(f"Log write error: {e}")
    
    
    def trigger_alert(self, level: str, source: str, message: str, speak_message: str = None):
        """Unified alert handling with cooldown"""
        
        # Check cooldown
        alert_key = f"{level}_{source}_{message}"
        current_time = time.time()
        
        with self.alert_lock:
            last_time = self.last_alert_time.get(alert_key, 0)
            if current_time - last_time < self.alert_cooldown:
                return  # Skip duplicate alert
            
            self.last_alert_time[alert_key] = current_time
            self.alert_count[level] = self.alert_count.get(level, 0) + 1
        
        # Console output with color
        color_map = {
            "CRITICAL": "\033[91m",  # Red
            "HIGH": "\033[93m",      # Yellow
            "MEDIUM": "\033[94m",    # Blue
            "WARNING": "\033[93m",   # Yellow
            "INFO": "\033[92m"       # Green
        }
        color = color_map.get(level, "\033[0m")
        reset = "\033[0m"
        
        icon_map = {
            "CRITICAL": " ",
            "HIGH": " ",
            "MEDIUM": " ",
            "WARNING": " ",
            "INFO": "ℹ️"
        }
        icon = icon_map.get(level, "⚡")
        
        print(f"{color}{icon} [{level}] {source}: {message}{reset}")
        
        # Log to file
        self.log_alert(level, source, message)
        
        # Play sound
        self.play_sound(level)
        
        # Speak alert for critical events
        if level in ["CRITICAL", "HIGH"] and speak_message:
            self.speak_alert(speak_message)
    
  
    # ROS2 CALLBACKS
    
    
    def on_ai_detections(self, msg: String):
        """Handle incoming AI detection alerts"""
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
            overall_threat = data.get("overall_threat", "NORMAL")
            
            # Track threat level change
            if overall_threat != self.current_threat_level:
                old_level = self.current_threat_level
                self.current_threat_level = overall_threat
                
                if overall_threat == "CRITICAL":
                    self.trigger_alert(
                        "CRITICAL", 
                        "AI_THREAT", 
                        f"Threat level escalated from {old_level} to CRITICAL",
                        "Critical threat detected. Immediate action required."
                    )
                elif overall_threat == "HIGH":
                    self.trigger_alert(
                        "HIGH",
                        "AI_THREAT",
                        f"Threat level elevated to HIGH",
                        "High threat level detected."
                    )
            
            # Process individual detections
            for det in detections:
                label = det.get("label", "Unknown")
                conf = det.get("confidence", 0.0)
                threat_level = det.get("threat_level", "NORMAL")
                det_type = det.get("type", "unknown")
                
                # WEAPON DETECTION - Highest priority
                if " " in label or det_type == "weapon":
                    weapon_type = label.replace(" ", "").strip()
                    self.armed_person_detected = True
                    self.trigger_alert(
                        "CRITICAL",
                        "WEAPON_DETECT",
                        f"ARMED PERSON DETECTED: {weapon_type} (Confidence: {conf*100:.1f}%)",
                        f"Warning! Armed person detected. {weapon_type} identified."
                    )
                
                # SUSPICIOUS ACTIVITY
                elif det_type == "activity" and threat_level in ["HIGH", "CRITICAL"]:
                    activity = label.replace(" ", "").strip()
                    if conf >= 0.7:
                        self.trigger_alert(
                            "HIGH",
                            "ACTIVITY",
                            f"Suspicious activity: {activity} (Confidence: {conf*100:.1f}%)",
                            f"Suspicious activity detected: {activity}"
                        )
                
                # CROWD DETECTION
                elif "👥" in label or det_type == "crowd":
                    person_count = int(''.join(filter(str.isdigit, label)) or 0)
                    if person_count > 20:
                        self.trigger_alert(
                            "HIGH",
                            "CROWD",
                            f"Large crowd detected: {person_count} persons",
                            f"Large crowd of {person_count} persons detected"
                        )
                    elif person_count > 10:
                        self.trigger_alert(
                            "MEDIUM",
                            "CROWD",
                            f"Crowd detected: {person_count} persons"
                        )
                
                # HIGH CONFIDENCE UNKNOWN OBJECT
                elif conf >= 0.85 and threat_level in ["HIGH", "CRITICAL"]:
                    self.trigger_alert(
                        "HIGH",
                        "AI_DETECT",
                        f"High-confidence target: {label} ({conf*100:.1f}%)"
                    )
        
        except Exception as e:
            self.get_logger().warning(f"AI detection parse error: {e}")
    
    
    def on_failsafe_event(self, msg: String):
        """Handle system-level failsafe triggers"""
        try:
            event = json.loads(msg.data)
            reason = event.get("reason", "UNKNOWN")
            action = event.get("action", "NONE")
            severity = event.get("severity", "HIGH")
            
            self.trigger_alert(
                severity,
                "FAILSAFE",
                f"Emergency: {reason} → Action: {action}",
                f"Failsafe triggered: {reason}"
            )
        
        except Exception as e:
            self.get_logger().warning(f"Failsafe event error: {e}")
    
    
    def on_flight_status(self, msg: String):
        """Monitor for general system warnings"""
        try:
            data = json.loads(msg.data)
            battery_data = data.get("battery", {})
            battery = battery_data.get("battery_remaining", None)
            
            if battery is not None:
                if battery <= 10:
                    self.trigger_alert(
                        "CRITICAL",
                        "BATTERY",
                        f"CRITICAL battery level: {battery}%",
                        f"Critical battery warning. {battery} percent remaining."
                    )
                elif battery <= 20:
                    self.trigger_alert(
                        "HIGH",
                        "BATTERY",
                        f"Low battery level: {battery}%",
                        f"Low battery. {battery} percent remaining."
                    )
            
            # Check GPS
            gps_ok = data.get("gps_ok", True)
            if not gps_ok:
                self.trigger_alert(
                    "HIGH",
                    "GPS",
                    "GPS signal lost or degraded",
                    "GPS signal lost"
                )
        
        except Exception:
            pass
    
    
    
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info(" AlertSystem shutting down")
        self.get_logger().info(f"   Total alerts: CRITICAL={self.alert_count.get('CRITICAL', 0)}, "
                             f"HIGH={self.alert_count.get('HIGH', 0)}, "
                             f"MEDIUM={self.alert_count.get('MEDIUM', 0)}")
        
        if self.tts_engine:
            try:
                self.tts_engine.stop()
            except:
                pass
        
        if PYGAME_AVAILABLE:
            try:
                pygame.mixer.quit()
            except:
                pass
        
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AlertSystem()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
