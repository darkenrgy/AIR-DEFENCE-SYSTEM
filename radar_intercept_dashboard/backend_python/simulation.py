# simulation.py - simulation engine with control for start/stop scanning and multi-sensor support (placeholder)
from dataclasses import dataclass
from typing import List
import math, threading, time, random

@dataclass
class SimObject:
    id: int
    type: str
    position: List[float]  # [x,y]
    velocity: List[float]  # [vx,vy]
    cross_section: float
    active: bool = True

    def dict(self):
        return {
            "id": self.id,
            "type": self.type,
            "position": [float(self.position[0]), float(self.position[1])],
            "velocity": [float(self.velocity[0]), float(self.velocity[1])],
            "cross_section": float(self.cross_section),
            "speed": float(math.hypot(self.velocity[0], self.velocity[1])),
            "active": self.active
        }

class Simulation:
    def __init__(self):
        self._objects = []
        self._lock = threading.Lock()
        self._id = 1
        self._running = False
        self._thread = None
        self.interceptors_sent = 0
        self.sensor_count = 1
    def next_id(self):
        with self._lock:
            i = self._id
            self._id += 1
        return i
    def add_object(self, obj: SimObject):
        with self._lock:
            self._objects.append(obj)
    def remove_object(self, obj_id:int):
        with self._lock:
            self._objects = [o for o in self._objects if o.id!=obj_id]
    def step(self, dt=0.5):
        with self._lock:
            for o in self._objects:
                if not o.active: continue
                o.position[0] += o.velocity[0] * dt
                o.position[1] += o.velocity[1] * dt
                if abs(o.position[0])>10000 or abs(o.position[1])>10000:
                    o.active = False
    def get_snapshot(self):
        with self._lock:
            return [o for o in self._objects if o.active]
    def get_by_id(self, obj_id):
        with self._lock:
            for o in self._objects:
                if o.id==obj_id: return o
            return None
    def start_auto(self, dt=0.5):
        if self._running:
            return
        self._running = True
        def loop():
            while self._running:
                self.step(dt)
                time.sleep(dt)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()
    def stop_auto(self):
        self._running = False
    def send_interceptor(self):
        with self._lock:
            self.interceptors_sent += 1
    def add_sensor(self):
        with self._lock:
            self.sensor_count += 1
    def get_stats(self):
        with self._lock:
            return {
                'num_objects': len([o for o in self._objects if o.active]),
                'interceptors_sent': self.interceptors_sent,
                'sensor_count': self.sensor_count
            }

sim = Simulation()

# seed objects
sim.add_object(SimObject(id=sim.next_id(), type='missile', position=[-1200,900], velocity=[300,-140], cross_section=0.8))
sim.add_object(SimObject(id=sim.next_id(), type='drone',   position=[-800,600],  velocity=[140,-60],  cross_section=2.5))
sim.add_object(SimObject(id=sim.next_id(), type='plane',   position=[-2000,1500],velocity=[400,-30],  cross_section=20.0))
