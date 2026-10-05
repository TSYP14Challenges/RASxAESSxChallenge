#!/usr/bin/env python3
"""The working bounded route animation, with optional facing at victim approaches."""
from bisect import bisect_right
import math

from step1_control import wrap
from step1_timeline import Timeline, model_poses


class MissionTimeline(Timeline):
    def __init__(self, waypoints, config, seconds, maximum_speed=None):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('Motion duration must be positive')
        self.waypoints, self.start = waypoints, tuple(config['start'])
        self.maximum_speed = min(maximum_speed or config['linear_speed'],
                                 .25*config['preview_frame_rate'])
        self.turn_speed = config['preview_turn_speed']
        self.separation, self.wheel_radius = .50, .11
        self.phases, self.ends, self.arrivals = [], [], []
        specs, heading, total_distance, total_turn = [], self.start[2], 0.0, 0.0
        for index, (a, b) in enumerate(zip(waypoints, waypoints[1:]), 1):
            length = math.hypot(b['x']-a['x'], b['y']-a['y'])
            if length <= 0:
                raise ValueError('Consecutive duplicate waypoints')
            change = wrap(math.atan2(b['y']-a['y'], b['x']-a['x'])-heading)
            if abs(change) > 1e-10:
                specs.append(('turn', a, a, heading, change, 0.0, index, False))
                total_turn += abs(change)
            heading += change
            face = wrap(b.get('face_yaw', heading)-heading)
            specs.append(('move', a, b, heading, 0.0, length, index, abs(face) <= 1e-10))
            total_distance += length
            if abs(face) > 1e-10:
                specs.append(('turn', b, b, heading, face, 0.0, index, True))
                total_turn += abs(face)
                heading += face
        final_change = wrap(self.start[2]+math.pi-heading)
        if abs(final_change) > 1e-10:
            specs.append(('turn', waypoints[-1], waypoints[-1], heading,
                          final_change, 0.0, len(waypoints)-1, False))
            total_turn += abs(final_change)
        turn_seconds = total_turn/self.turn_speed
        self.minimum_seconds = turn_seconds+total_distance/self.maximum_speed
        if seconds < self.minimum_seconds:
            raise ValueError(f'Motion needs at least {self.minimum_seconds:.1f} seconds at this speed')
        self.linear_speed = total_distance/(seconds-turn_seconds)
        current, travelled = 0.0, 0.0
        for kind, a, b, yaw, angle, distance, index, arrival in specs:
            duration = abs(angle)/self.turn_speed if kind == 'turn' else distance/self.linear_speed
            p = {'kind': kind, 'start': current, 'end': current+duration,
                 'a': a, 'b': b, 'yaw': yaw, 'angle': angle, 'distance': distance,
                 'travelled': travelled, 'index': index}
            self.phases.append(p)
            self.ends.append(p['end'])
            if arrival:
                self.arrivals.append((p['end'], index, b.get('kind'), b.get('name', '')))
            current += duration
            travelled += distance
        self.duration, self.distance = current, travelled
        self.final_yaw = heading+final_change
