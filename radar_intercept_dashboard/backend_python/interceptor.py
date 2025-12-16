# interceptor.py - compute multiple interception candidate points along linear path
import math

def compute_interception_points(target, interceptor_speed=300.0, max_checks=8, dt_step=0.5):
    px, py = target.position
    vx, vy = target.velocity
    candidates = []
    for i in range(1, max_checks+1):
        t_future = i * dt_step * 1.0
        tx = px + vx * t_future
        ty = py + vy * t_future
        dist = math.hypot(tx, ty)
        if interceptor_speed <= 0:
            continue
        interceptor_time = dist / interceptor_speed
        if interceptor_time <= t_future + 2.0:
            candidates.append({
                'time': t_future,
                'point': [tx, ty],
                'interceptor_time': interceptor_time
            })
    candidates.sort(key=lambda x: x['time'])
    return candidates
