#!/usr/bin/env python3
"""Timed movement along the complete safe route, independent of wheel dynamics."""
from bisect import bisect_right
import math

from step1_control import wrap


class Timeline:
    def __init__(self, waypoints, config, seconds, maximum_speed=None):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('Tour duration must be a positive number of seconds')
        self.waypoints = waypoints
        self.start = tuple(config['start'])
        self.maximum_speed = min(maximum_speed or config['linear_speed'],
                                 .25*config['preview_frame_rate'])
        self.turn_speed = config['preview_turn_speed']
        self.separation, self.wheel_radius = 0.50, 0.11
        self.phases, self.ends, self.arrivals = [], [], []
        specs, heading = [], self.start[2]
        total_distance, total_turn = 0.0, 0.0
        for index, (a, b) in enumerate(zip(waypoints, waypoints[1:]), 1):
            length = math.hypot(b['x']-a['x'], b['y']-a['y'])
            if length <= 0:
                raise ValueError('The route contains duplicate consecutive waypoints')
            target_heading = math.atan2(b['y']-a['y'], b['x']-a['x'])
            change = wrap(target_heading-heading)
            if abs(change) > 1e-10:
                specs.append(('turn', a, a, heading, change, 0.0, index))
                total_turn += abs(change)
            heading += change
            specs.append(('move', a, b, heading, 0.0, length, index))
            total_distance += length
        final_change = wrap(self.start[2]+math.pi-heading)
        if abs(final_change) > 1e-10:
            specs.append(('turn', waypoints[-1], waypoints[-1], heading,
                          final_change, 0.0, len(waypoints)-1))
            total_turn += abs(final_change)
        turn_seconds = total_turn/self.turn_speed
        self.minimum_seconds = turn_seconds+total_distance/self.maximum_speed
        if seconds < self.minimum_seconds:
            raise ValueError(f'This complete route needs at least {self.minimum_seconds:.1f} seconds '
                             f'at the selected maximum speed; increase --seconds')
        self.linear_speed = total_distance/(seconds-turn_seconds)
        current, travelled = 0.0, 0.0
        for kind, a, b, yaw, angle, distance, index in specs:
            duration = abs(angle)/self.turn_speed if kind == 'turn' else distance/self.linear_speed
            phase = {'kind': kind, 'start': current, 'end': current+duration,
                     'a': a, 'b': b, 'yaw': yaw, 'angle': angle, 'distance': distance,
                     'travelled': travelled, 'index': index}
            self.phases.append(phase)
            self.ends.append(phase['end'])
            if kind == 'move':
                self.arrivals.append((phase['end'], index, b.get('kind'), b.get('name', '')))
            current += duration
            travelled += distance
        self.duration, self.distance = current, travelled
        self.final_yaw = heading+final_change

    def sample(self, elapsed):
        elapsed = max(0.0, min(elapsed, self.duration))
        if elapsed >= self.duration-1e-9:
            last = self.waypoints[-1]
            x, y, yaw, distance = last['x'], last['y'], self.final_yaw, self.distance
            index, mode, linear, angular = len(self.waypoints)-1, 'finished', 0.0, 0.0
        else:
            p = self.phases[min(bisect_right(self.ends, elapsed), len(self.phases)-1)]
            fraction = max(0.0, min(1.0, (elapsed-p['start'])/(p['end']-p['start'])))
            x = p['a']['x']+(p['b']['x']-p['a']['x'])*fraction
            y = p['a']['y']+(p['b']['y']-p['a']['y'])*fraction
            yaw = p['yaw']+p['angle']*fraction
            distance = p['travelled']+p['distance']*fraction
            index, mode = p['index'], p['kind']
            linear = self.linear_speed if mode == 'move' else 0.0
            angular = math.copysign(self.turn_speed, p['angle']) if mode == 'turn' else 0.0
        change = yaw-self.start[2]
        return {'x': x, 'y': y, 'yaw': yaw, 'distance': distance, 'index': index,
                'mode': mode, 'linear': linear, 'angular': angular,
                'left_spin': (distance-self.separation*change/2)/self.wheel_radius,
                'right_spin': (distance+self.separation*change/2)/self.wheel_radius}

    def advance(self, elapsed, wall_delta):
        """Never skip a waypoint, or jump more than 25 cm / 0.2 rad in a frame."""
        if elapsed >= self.duration-1e-9:
            return self.duration
        p = self.phases[min(bisect_right(self.ends, elapsed), len(self.phases)-1)]
        limit = .25/self.linear_speed if p['kind'] == 'move' else .20/self.turn_speed
        return min(p['end'], elapsed+max(0.0, min(wall_delta, limit)))


def model_poses(state, model_name, separation=.50):
    """World poses for the cube and two separately animated wheel models."""
    x, y, yaw = state['x'], state['y'], state['yaw']
    half = yaw/2
    poses = [(model_name, x, y, .005, 0.0, 0.0, math.sin(half), math.cos(half))]
    for side, direction in (('left', 1), ('right', -1)):
        spin = state[side+'_spin']/2
        poses.append((model_name+'_'+side+'_wheel',
                      x-direction*separation*math.sin(yaw)/2,
                      y+direction*separation*math.cos(yaw)/2, .115,
                      -math.sin(half)*math.sin(spin), math.cos(half)*math.sin(spin),
                      math.sin(half)*math.cos(spin), math.cos(half)*math.cos(spin)))
    return poses
