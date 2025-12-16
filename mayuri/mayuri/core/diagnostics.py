import os
import json
import time
import psutil
import logging
import statistics
from pathlib import Path
from datetime import datetime, timedelta
from threading import Thread

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    # Standalone fallback
    rclpy = None
    Node = object
    String = None



class DiagnosticsEngine(Node if rclpy else object):
    def __init__(self, log_dir="logs/system", interval=30):
        if rclpy:
            super().__init__("diagnostics_engine")

        self.log_dir = Path(log_dir)
        self.interval = interval
        self.active = True

        # Setup logging
        log_file = self.log_dir / "diagnostics_report.log"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s]: %(message)s",
        )

        # ROS2 publisher
        if rclpy:
            self.report_pub = self.create_publisher(String, "/mayuri/system/diagnostics", 10)
            self.alert_pub = self.create_publisher(String, "/mayuri/system/alerts", 10)

        logging.info("DiagnosticsEngine initialized")
        print("🩺 Diagnostics Engine running...")

        # Launch background thread
        thread = Thread(target=self._run_loop, daemon=True)
        thread.start()

   
    def _load_recent_logs(self, minutes=10):
        metrics = []
        try:
            sys_log = self.log_dir / "system_monitor.log"
            if not sys_log.exists():
                return metrics

            cutoff = datetime.utcnow() - timedelta(minutes=minutes)
            with open(sys_log, "r") as f:
                for line in f:
                    if "{" not in line:
                        continue
                    try:
                        record = json.loads(line.split("INFO:")[1].strip())
                        ts = datetime.fromisoformat(record["timestamp"])
                        if ts > cutoff:
                            metrics.append(record)
                    except Exception:
                        continue
        except Exception as e:
            logging.error(f"Error loading logs: {e}")
        return metrics

    
    def _analyze_metrics(self, metrics):
        if not metrics:
            return None

        cpu = [m["cpu_usage_percent"] for m in metrics if "cpu_usage_percent" in m]
        mem = [m["memory_usage_percent"] for m in metrics if "memory_usage_percent" in m]
        temps = [m.get("temperature_c") for m in metrics if m.get("temperature_c") is not None]

        avg_cpu = round(statistics.mean(cpu), 2) if cpu else 0
        avg_mem = round(statistics.mean(mem), 2) if mem else 0
        avg_temp = round(statistics.mean(temps), 2) if temps else None

        # Detect anomalies
        warnings = []
        if avg_cpu > 85:
            warnings.append(" High average CPU load detected.")
        if avg_mem > 90:
            warnings.append("High memory usage.")
        if avg_temp and avg_temp > 75:
            warnings.append(" System temperature above safe range.")
        if len(cpu) > 5 and max(cpu) - min(cpu) > 60:
            warnings.append("⚡ CPU usage fluctuations detected (instability risk).")

        # Predictive failure scoring
        stability_score = 100 - ((avg_cpu / 2) + (avg_mem / 3))
        stability_score = max(min(stability_score, 100), 0)

        report = {
            "timestamp": datetime.utcnow().isoformat(),
            "avg_cpu": avg_cpu,
            "avg_mem": avg_mem,
            "avg_temp": avg_temp,
            "stability_score": round(stability_score, 2),
            "warnings": warnings,
        }

        return report

    
    def _generate_report(self, report):
        if not report:
            return

        msg = f"🩺 Diagnostics Report | CPU: {report['avg_cpu']}% | MEM: {report['avg_mem']}% | TEMP: {report.get('avg_temp', 'N/A')}°C | STABILITY: {report['stability_score']}%"
        logging.info(msg)
        print(msg)

        if rclpy and String:
            self.report_pub.publish(String(data=json.dumps(report)))

        # Send alert if severe issues found
        if report["stability_score"] < 60 or report["warnings"]:
            self._trigger_alert(report)

    
    def _trigger_alert(self, report):
        alert_msg = {
            "timestamp": report["timestamp"],
            "type": "SYSTEM_ALERT",
            "details": report["warnings"],
        }
        alert_json = json.dumps(alert_msg)
        logging.warning(alert_json)

        print(f" ALERT: {alert_msg['details']}")
        if rclpy and String:
            self.alert_pub.publish(String(data=alert_json))

    
    def _run_loop(self):
        while self.active:
            try:
                metrics = self._load_recent_logs(minutes=5)
                report = self._analyze_metrics(metrics)
                self._generate_report(report)
                time.sleep(self.interval)
            except Exception as e:
                logging.error(f"Diagnostics loop error: {e}")
                time.sleep(self.interval)

    
    def stop(self):
        self.active = False
        logging.info("DiagnosticsEngine stopped")
        print(" Diagnostics engine stopped.")


def run_diagnostics(interval=30):
    if rclpy:
        rclpy.init()
        node = DiagnosticsEngine(interval=interval)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        diag = DiagnosticsEngine(interval=interval)
        try:
            while diag.active:
                time.sleep(1)
        except KeyboardInterrupt:
            diag.stop()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MAYURI Diagnostics Engine")
    parser.add_argument("--interval", type=int, default=30, help="Diagnostics check interval (seconds)")
    args = parser.parse_args()

    run_diagnostics(args.interval)
