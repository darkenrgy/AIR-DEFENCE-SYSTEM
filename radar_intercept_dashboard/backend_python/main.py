# main.py - FastAPI backend for radar intercept dashboard (v2)
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from simulation import sim, SimObject
from interceptor import compute_interception_points
from classifier import classify_by_cross_section
from sensor_integration import fuse_reports
from typing import List
import random, copy

app = FastAPI(title='Radar Intercept Dashboard Backend v2')

class ManualCreate(BaseModel):
    type: str = 'drone'
    position: List[float]
    velocity: List[float]
    cross_section: float = 1.0

@app.get('/objects')
def get_objects():
    return {'objects': [o.dict() for o in sim.get_snapshot()]}

@app.get('/stats')
def get_stats():
    return sim.get_stats()

@app.post('/start_scan')
def start_scan():
    sim.start_auto(dt=0.5)
    return {'status':'scanning_started'}

@app.post('/stop_scan')
def stop_scan():
    sim.stop_auto()
    return {'status':'scanning_stopped'}

@app.post('/create_manual')
def create_manual(obj: ManualCreate):
    s = SimObject(id=sim.next_id(), type=obj.type, position=obj.position, velocity=obj.velocity, cross_section=obj.cross_section)
    sim.add_object(s)
    return {'status':'ok', 'object': s.dict()}

@app.post('/predict_intercept/{obj_id}')
def predict_intercept(obj_id:int, interceptor_speed: float = 300.0):
    target = sim.get_by_id(obj_id)
    if target is None:
        raise HTTPException(status_code=404, detail='object not found')
    candidates = compute_interception_points(target, interceptor_speed=interceptor_speed)
    for c in candidates:
        c['probability'] = max(0.05, 1.0 - abs(c['interceptor_time'] - c['time'])/ (c['time']+1.0))
    # simulate sending interceptor for best candidate and track
    if candidates:
        sim.send_interceptor()
    return {'object_id': obj_id, 'candidates': candidates}

@app.post('/add_sensor')
def add_sensor(info: dict):
    sim.add_sensor()
    return {'status':'sensor_added', 'info': info}

class SensorReport(BaseModel):
    sensor_id: int
    objects: List[dict]

@app.post('/simulate_multi_sensor')
def simulate_multi_sensor(reports: List[SensorReport]):
    # create simple sensor_reports lists compatible with fuse_reports
    sensor_reports = []
    for r in reports:
        lst = []
        for o in r.objects:
            lst.append({'id': o.get('id'), 'position': o.get('position'), 'velocity': o.get('velocity'), 'cross_section': o.get('cross_section',1.0)})
        sensor_reports.append(lst)
    fused = fuse_reports(sensor_reports)
    # apply fused positions into simulation (replace existing object positions)
    for f in fused:
        existing = sim.get_by_id(f['id'])
        if existing:
            existing.position = f['position']
            existing.velocity = f['velocity']
    return {'status':'fused', 'fused_count': len(fused)}
