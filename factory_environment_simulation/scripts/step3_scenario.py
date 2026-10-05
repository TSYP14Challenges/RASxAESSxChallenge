#!/usr/bin/env python3
"""Small second-robot scenario: visit event tiles and leave network tiles alone."""
from copy import deepcopy
import json
import math

from step1_map import GridMap, Rectangle, build_grid, read_geometry
from step2_events import MissionEvents, PACKAGE, load_config as writer_config, read_events
from step2_timeline import MissionTimeline


def load_config():
    return {**writer_config(), **json.loads((PACKAGE/'step3/config.json').read_text())}


class AccessibleWriterEvents(MissionEvents):
    """Keep the writer's route; put orange/white tiles on robot-accessible floor."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.action_grid = GridMap.load(PACKAGE/'step2/map.json')
        self.action_reachable = set()
        self.require_executor_space = False

    def placement(self, state, kind, target=None):
        self.require_executor_space = kind in ('fire', 'victim')
        try:
            if self.require_executor_space:
                gray = load_config()
                self.action_grid = action_grid(gray, self.records)
                self.action_reachable = self.action_grid.reachable(
                    self.action_grid.index(*gray['start'][:2]))
            return super().placement(state, kind, target)
        finally:
            self.require_executor_space = False

    def tile_safe(self, xy, state=None, require_link=False):
        if not super().tile_safe(xy, state, require_link):
            return False
        if not self.require_executor_space:
            return True
        return self.action_grid.index(*xy) in self.action_reachable


def validate_handoff(writer_result, records):
    if not writer_result.get('completed') or writer_result.get('finish_error_metres', math.inf) > .025:
        raise RuntimeError('The writer must return outside and finish before the gray robot enters')
    expected = {e['id']: e['kind'] for e in read_events(writer_config())}
    targets = [b for b in records if b.get('kind') in ('fire', 'victim')]
    if len(targets) != len(expected) or {b['id']: b['kind'] for b in targets} != expected:
        raise RuntimeError('The gray robot requires all six fire and two victim beacons')
    if len({b['beacon_name'] for b in records}) != len(records):
        raise RuntimeError('Duplicate beacon names in writer handoff')
    for record in records:
        if not all(math.isfinite(v) for v in record['position']) or len(record['position']) != 3:
            raise RuntimeError('Invalid beacon position in writer handoff')
    return targets


def action_grid(config, records):
    floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    for record in records:
        if record['kind'] in ('gateway', 'range'):
            x, y = record['position'][:2]
            obstacles.append(Rectangle(record['beacon_name'], [x-.05, y-.05, x+.05, y+.05]))
    wx, wy, _ = writer_config()['start']
    obstacles.append(Rectangle('parked_writer', [wx-.30, wy-.30, wx+.30, wy+.30]))
    nx, ny, _ = writer_config()['outside_gateway_position']
    obstacles.append(Rectangle('outside_gateway', [nx-.225, ny-.175, nx+.225, ny+.175]))
    return build_grid(floors, obstacles, fires, config)


def connection_path(grid, start, finish):
    # Keep a small clearance from grid corners when the yellow layout changes.
    clearance = .02
    if grid.line_safe(start, finish, clearance):
        return [start, finish]
    raw = [grid.point(cell) for cell in grid.shortest_path(grid.index(*start), grid.index(*finish))]
    # Keep the exact beacon centre, rather than rounding its final position to a cell.
    points = [tuple(start), *raw, tuple(finish)]
    unique = [points[0]]
    for point in points[1:]:
        if math.dist(point, unique[-1]) > 1e-8:
            unique.append(point)
    points = grid.simplify(unique, tracking_buffer=clearance)
    if not all(grid.line_safe(a, b) for a, b in zip(points, points[1:])):
        raise RuntimeError('No safe route to the exact beacon centre')
    return points


def plan_executor(config, records, writer_result):
    remaining = deepcopy(validate_handoff(writer_result, records))
    grid = action_grid(config, records)
    current = tuple(config['start'][:2])
    if not grid.safe(*current):
        raise RuntimeError('Gray robot start is blocked')
    reachable = grid.reachable(grid.index(*current))
    for record in remaining:
        if grid.index(*record['position'][:2]) not in reachable:
            raise RuntimeError('Gray robot cannot stand above '+record['beacon_name'])
    events = {e['id']: e for e in read_events(writer_config())}
    waypoints = [{'x': current[0], 'y': current[1], 'kind': 'start', 'name': 'Gray robot outside'}]
    actions = []
    while remaining:
        # Visit all victims before fires, choosing the nearest target in each group.
        target = min(remaining, key=lambda b: (
            0 if b['kind'] == 'victim' else 1,
            math.dist(current, b['position'][:2]), b['id']))
        finish = tuple(target['position'][:2])
        for x, y in connection_path(grid, current, finish)[1:]:
            waypoints.append({'x': x, 'y': y, 'kind': 'connection', 'name': 'To '+target['id']})
        centre = events[target['id']]['centre']
        waypoints[-1].update(kind='action', name=target['id'],
                             face_yaw=math.atan2(centre[1]-finish[1], centre[0]-finish[0]))
        target['event_centre'] = centre
        if target['kind'] == 'victim':
            target['victim_status'] = 'alive' if target['id'] == 'victim_alive' else 'dead'
        actions.append(target)
        current = finish
        remaining.remove(next(b for b in remaining if b['id'] == target['id']))
    for x, y in connection_path(grid, current, config['start'][:2])[1:]:
        waypoints.append({'x': x, 'y': y, 'kind': 'return', 'name': 'Gray robot returns outside'})
    estimate = MissionTimeline(waypoints, config, 600., config['maximum_speed'])
    seconds = max(config['motion_seconds'], estimate.minimum_seconds+1.)
    timeline = MissionTimeline(waypoints, config, seconds, config['maximum_speed'])
    plan = {'waypoints': waypoints, 'actions': actions,
            'ignored_beacons': [b['beacon_name'] for b in records if b['kind'] in ('gateway', 'range')],
            'motion_seconds': seconds, 'distance_metres': round(timeline.distance, 3)}
    return plan, grid


def ball_point(start, finish, fraction, arc_height):
    if not 0 <= fraction <= 1:
        raise ValueError('Ball flight fraction must be between zero and one')
    return [start[i]+(finish[i]-start[i])*fraction+
            (4*arc_height*fraction*(1-fraction) if i == 2 else 0.) for i in range(3)]
