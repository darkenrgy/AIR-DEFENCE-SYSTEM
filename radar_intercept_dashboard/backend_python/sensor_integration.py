# sensor_integration.py - simulate combining multiple sensor reports into single object list
# For demo: each sensor reports slightly noisy position for each object; backend fuses by averaging
import random, copy

def fuse_reports(sensor_reports):
    # sensor_reports: list of lists of objects (dicts with id, position, velocity, cross_section)
    fused = {}
    for report in sensor_reports:
        for o in report:
            oid = o['id']
            if oid not in fused:
                fused[oid] = {'id': oid, 'positions': [], 'velocities': [], 'cross_section': o.get('cross_section', 1.0)}
            fused[oid]['positions'].append(o['position'])
            fused[oid]['velocities'].append(o['velocity'])
    result = []
    for oid, data in fused.items():
        # average positions and velocities
        avg_pos = [sum(p[i] for p in data['positions'])/len(data['positions']) for i in (0,1)]
        avg_vel = [sum(v[i] for v in data['velocities'])/len(data['velocities']) for i in (0,1)]
        result.append({'id': oid, 'position': avg_pos, 'velocity': avg_vel, 'cross_section': data['cross_section']})
    return result
