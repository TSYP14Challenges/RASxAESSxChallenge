#!/usr/bin/env python3
"""Known-position event detection and safe, permanent beacon placement.

Lengths are metres. Detection is a horizontal edge-to-edge distance, using
the actual chassis/wheel footprint and victim collisions. Fires use the
existing 1.1 m excluded fire region. No camera or radio propagation model.
"""
import json
import math
from pathlib import Path

from step1_map import Rectangle, read_geometry
from step2_network import BeaconNetwork, ENTRANCE_BEACON

PACKAGE = Path(__file__).resolve().parents[1]
COLORS = {'gateway': [0.08, 0.35, 1.0, 1], 'fire': [1, 0.45, 0.0, 1],
          'victim': [1.0, 1.0, 1.0, 1], 'range': [1, 0.83, 0.02, 1]}
ROBOT_BOXES = [(-.26, -.20, .14, .20), (-.11, .225, .11, .275),
               (-.11, -.275, .11, -.225), (-.275, -.045, -.185, .045)]


def polygon(bounds):
    x0, y0, x1, y1 = bounds
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def footprint(state):
    c, s = math.cos(state['yaw']), math.sin(state['yaw'])
    return [[(state['x']+c*x-s*y, state['y']+s*x+c*y) for x, y in polygon(b)]
            for b in ROBOT_BOXES]


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    denom = dx*dx+dy*dy
    t = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom)) if denom else 0.0
    return math.dist(p, (a[0]+t*dx, a[1]+t*dy))


def inside(p, shape):
    cross = [(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
             for a, b in zip(shape, shape[1:]+shape[:1])]
    return min(cross) >= -1e-10 or max(cross) <= 1e-10


def polygon_distance(a, b):
    # Convex separating-axis test includes intersecting edges with no contained vertex.
    separated = False
    for shape in (a, b):
        for p, q in zip(shape, shape[1:]+shape[:1]):
            nx, ny = -(q[1]-p[1]), q[0]-p[0]
            aa = [nx*x+ny*y for x, y in a]
            bb = [nx*x+ny*y for x, y in b]
            if max(aa) < min(bb)-1e-10 or max(bb) < min(aa)-1e-10:
                separated = True
    if not separated:
        return 0.0
    return min(point_segment(p, x, y) for shape, other in ((a, b), (b, a))
               for p in shape for x, y in zip(other, other[1:]+other[:1]))


def distance_to_point(p, shapes):
    return min(0.0 if inside(p, shape) else
               min(point_segment(p, a, b) for a, b in zip(shape, shape[1:]+shape[:1]))
               for shape in shapes)


def event_gap(state, event):
    shapes = footprint(state)
    if event['kind'] == 'fire':
        return max(0.0, distance_to_point(event['centre'], shapes)-event['radius'])
    if event['kind'] == 'victim':
        return min(polygon_distance(robot, polygon(b))
                   for robot in shapes for b in event['bounds'])
    return math.dist((state['x'], state['y']), event['centre'])


def read_events(config):
    _, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    events = [{'id': name, 'kind': 'fire', 'centre': [x, y],
               'radius': config['fire_radius'], 'distance': config['fire_detection_metres']}
              for name, x, y in fires]
    names = sorted({o.name for o in obstacles if o.name.startswith('victim_')})
    for name in names:
        bounds = [o.bounds for o in obstacles if o.name == name]
        events.append({'id': name, 'kind': 'victim', 'bounds': bounds,
                       'centre': [(min(b[0] for b in bounds)+max(b[2] for b in bounds))/2,
                                  (min(b[1] for b in bounds)+max(b[3] for b in bounds))/2],
                       'distance': config['victim_detection_metres']})
    return events


def load_config():
    source = json.loads((PACKAGE/'step1/config.json').read_text())
    source.update(json.loads((PACKAGE/'step2/config.json').read_text()))
    return source


class MissionEvents:
    def __init__(self, config, events, floors, obstacles, fires):
        self.config, self.events = config, events
        self.floors, self.obstacles, self.fires = floors, obstacles, fires
        self.beacons, self.records = [], []
        self.done = set()
        self.range_count = 0
        self.network = BeaconNetwork(config)

    def nearest(self, state):
        if not self.beacons:
            return None, float('inf')
        xy = state['x'], state['y']
        beacon = min(self.beacons, key=lambda b: math.dist(xy, b['position'][:2]))
        return beacon, math.dist(xy, beacon['position'][:2])

    def tile_safe(self, xy, state=None, require_link=False):
        # Circumscribed tile circle protects the complete square, not just its centre.
        r = math.hypot(*self.config['beacon_size_metres'][:2])/2+.005
        x, y = xy
        for i in range(16):
            px, py = x+r*math.cos(i*math.pi/8), y+r*math.sin(i*math.pi/8)
            if not any(f.contains(px, py) for f in self.floors):
                return False
        if any(o.distance(x, y) < r+.025 for o in self.obstacles):
            return False
        if any(math.dist(xy, (fx, fy)) < self.config['fire_radius']+r+.025
               for _, fx, fy in self.fires):
            return False
        if state and distance_to_point(xy, footprint(state)) < r+.04:
            return False
        if any(math.dist(xy, b['position'][:2]) < 2*r+.03 for b in self.beacons):
            return False
        if require_link and not any(math.dist(xy, b['position'][:2]) <=
                                    self.config['communication_range_metres'] for b in self.beacons):
            return False
        return True

    def placement(self, state, kind, target=None):
        if kind == 'gateway':
            point = self.config['entrance_beacon_position'][:2]
            if not self.tile_safe(point, state):
                raise RuntimeError('Entrance beacon position is not clear')
            return list(self.config['entrance_beacon_position'])
        nearest, _ = self.nearest(state)
        # A relay goes back toward the previous beacon, restoring its radio link.
        # Other tiles prefer the robot's side, so they remain visible beside it.
        if kind == 'range' and nearest:
            preferred = math.atan2(nearest['position'][1]-state['y'],
                                   nearest['position'][0]-state['x'])
        elif kind == 'fire' and target:
            preferred = math.atan2(target['centre'][1]-state['y'],
                                   target['centre'][0]-state['x'])
        else:
            preferred = state['yaw']+math.pi/2
        offsets = [0, math.pi, math.pi/2, -math.pi/2] + [i*math.pi/12 for i in range(24)]
        for radius in (.43, .50, .60, .70):
            for delta in offsets:
                angle = preferred+delta
                point = state['x']+radius*math.cos(angle), state['y']+radius*math.sin(angle)
                if point[1] <= -12.0:
                    continue
                if self.tile_safe(point, state, require_link=True):
                    return [point[0], point[1], self.config['beacon_size_metres'][2]/2+.001]
        raise RuntimeError('No safe, connected floor position for '+kind+' beacon')

    def candidate(self, state):
        # Returned events remain pending until actual tile feedback is confirmed.
        if 'gateway' not in self.done:
            if math.dist((state['x'], state['y']), self.config['entrance_robot_stop']) <= .035:
                return {'id': 'gateway', 'kind': 'gateway', 'distance': 0.0,
                        'beacon_name': ENTRANCE_BEACON,
                        'position': self.placement(state, 'gateway')}
            return None
        if state['y'] <= -12.0:
            return None
        nearest, distance = self.nearest(state)
        if distance > self.config['communication_range_metres']:
            number = self.range_count+1
            if number > self.config['range_beacon_capacity']:
                raise RuntimeError('Range beacon capacity reached; increase it and run plan')
            return {'id': f'range_{number:02d}', 'kind': 'range', 'distance': distance,
                    'nearest_beacon': nearest['name'],
                    'beacon_name': f'step2_range_beacon_{number:02d}',
                    'position': self.placement(state, 'range')}
        for event in self.events:
            if event['id'] in self.done:
                continue
            distance = event_gap(state, event)
            if distance <= event['distance']+1e-8:
                result = {'id': event['id'], 'kind': event['kind'], 'distance': distance,
                        'threshold': event['distance'],
                        'beacon_name': 'step2_'+event['id']+'_beacon',
                        'position': self.placement(state, event['kind'], event)}
                if event['kind'] == 'victim':
                    result['victim_status'] = 'alive' if event['id'] == 'victim_alive' else 'dead'
                return result
        return None

    def commit(self, event, state, wall_elapsed=0.0, pause_seconds=1.0):
        if event['id'] in self.done:
            raise RuntimeError('Duplicate event '+event['id'])
        nearest, distance = self.nearest({'x': event['position'][0], 'y': event['position'][1]})
        if event['kind'] != 'gateway' and distance > self.config['communication_range_metres']+1e-8:
            raise RuntimeError('Dropped beacon would not connect to the entrance gateway')
        record = {**event, 'robot_pose': [state['x'], state['y'], state['yaw']],
                  'elapsed_wall_seconds': round(wall_elapsed, 3),
                  'pause_seconds': round(pause_seconds, 3),
                  'parent_beacon': None if nearest is None else nearest['name'],
                  'parent_distance_metres': None if nearest is None else round(distance, 5)}
        record['device_role'] = 'entrance_uplink' if event['kind'] == 'gateway' else 'event_beacon'
        # The mission calls commit only after the placed model is confirmed.
        # The entrance model contains the tile and cable in the same creation.
        self.network.register(record)
        if event['kind'] == 'gateway':
            self.network.connect_cable()
        delivery = self.network.send(record)
        record.update(gateway_delivery=delivery['status'], data_route=delivery['route'])
        self.records.append(record)
        self.done.add(event['id'])
        if event['kind'] == 'range':
            self.range_count += 1
        self.beacons.append({'name': event['beacon_name'], 'kind': event['kind'],
                             'position': event['position']})
        return record

    def require_complete(self):
        missing = ({'gateway'} | {e['id'] for e in self.events})-self.done
        if missing:
            raise RuntimeError('Missing beacon events: '+', '.join(sorted(missing)))
        self.network.require_received(self.records)
