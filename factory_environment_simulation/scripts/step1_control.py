#!/usr/bin/env python3
"""Small turn-and-drive waypoint follower, shared by Gazebo and offline validation."""
import math


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class Follower:
    def __init__(self, waypoints, config, grid):
        self.waypoints, self.config, self.grid = waypoints, config, grid
        self.index = 1
        self.finished = False
        self.mode = 'waiting'
        self.last_speed = 0.0

    def ramp_speed(self, target, dt):
        dt = min(0.1, max(0.0, dt))
        self.last_speed = clamp(target,
                                max(0.0, self.last_speed-self.config['linear_deceleration']*dt),
                                self.last_speed+self.config['linear_acceleration']*dt)
        return self.last_speed

    def command(self, x, y, yaw, dt=1/30):
        if not self.grid.safe(x, y):
            raise RuntimeError('Robot has left the safe corridor; stopped to preserve clearance.')
        if self.finished:
            return 0.0, 0.0
        while self.index < len(self.waypoints):
            target = self.waypoints[self.index]
            distance = math.hypot(target['x']-x, target['y']-y)
            if distance > self.config['waypoint_tolerance'] or self.last_speed > 0.04:
                break
            self.index += 1
        if self.index >= len(self.waypoints):
            # Face the entrance on return, then hold a zero velocity command.
            error = wrap(self.config['start'][2]+math.pi-yaw)
            if abs(error) < 0.045:
                self.finished, self.mode = True, 'finished'
                self.last_speed = 0.0
                return 0.0, 0.0
            self.mode = 'final turn'
            self.last_speed = 0.0
            return 0.0, clamp(self.config['heading_gain']*error,
                              -self.config['angular_speed'], self.config['angular_speed'])
        # Aim a short distance along the current straight segment. This corrects
        # sideways wheel slip without allowing long segments to bow into obstacles.
        previous = self.waypoints[self.index-1]
        dx, dy = target['x']-previous['x'], target['y']-previous['y']
        length = math.hypot(dx, dy)
        progress = clamp(((x-previous['x'])*dx+(y-previous['y'])*dy)/length, 0, length)
        aim = min(length, progress+self.config['lookahead_distance'])
        ax, ay = previous['x']+dx*aim/length, previous['y']+dy*aim/length
        heading = math.atan2(ay-y, ax-x)
        error = wrap(heading-yaw)
        angular = clamp(self.config['heading_gain']*error,
                        -self.config['angular_speed'], self.config['angular_speed'])
        if abs(error) > 0.12:
            # Brake before an in-place turn. Small steering corrections during
            # braking counter wheel slip along the current segment.
            if self.last_speed > 0.04:
                self.mode = 'braking before turn'
                speed = self.ramp_speed(0.0, dt)
                correction = clamp(angular,
                                   -self.config['lateral_acceleration']/max(speed, 0.1),
                                   self.config['lateral_acceleration']/max(speed, 0.1))
                return speed, correction if abs(error) < math.pi/2 else 0.0
            self.mode = 'turning'
            self.last_speed = 0.0
            return 0.0, angular
        # Reserve both stopping distance v²/(2a) and v*T for feedback / motor lag.
        deceleration = self.config['linear_deceleration']
        response = self.config['braking_response_seconds']
        braking_speed = math.sqrt((deceleration*response)**2+2*deceleration*distance) \
            - deceleration*response
        target_speed = min(self.config['linear_speed'], braking_speed,
                           self.config['waypoint_approach_gain']*distance)
        speed = self.ramp_speed(target_speed, dt)
        angular = clamp(angular, -self.config['lateral_acceleration']/max(speed, 0.1),
                        self.config['lateral_acceleration']/max(speed, 0.1))
        # Sample by travelled distance: high speeds must not jump blocked cells.
        for _ in range(9):
            safe = True
            horizon = 0.385
            samples = max(8, int(math.ceil(speed*horizon/0.05)))
            for i in range(1, samples+1):
                prediction_time = i*horizon/samples
                if abs(angular) < 1e-6:
                    px, py = x+speed*prediction_time*math.cos(yaw), y+speed*prediction_time*math.sin(yaw)
                else:
                    px = x+speed/angular*(math.sin(yaw+angular*prediction_time)-math.sin(yaw))
                    py = y-speed/angular*(math.cos(yaw+angular*prediction_time)-math.cos(yaw))
                if not self.grid.safe(px, py):
                    safe = False
                    break
            if safe:
                self.mode = 'driving'
                self.last_speed = speed
                return speed, angular
            speed *= 0.5
        raise RuntimeError('No safe forward command at this pose. Regenerate or inspect the route.')
