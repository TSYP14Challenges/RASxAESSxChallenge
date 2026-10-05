#!/usr/bin/env python3
"""Validate source geometry, complete aisle route, and closed-loop differential-drive motion.

This is an offline numerical check, not a Gazebo contact-physics or rendering test.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from build_step1 import aisle_segments
from step1_control import Follower
from step1_map import GridMap, read_geometry
from step1_settings import MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED

PACKAGE = Path(__file__).resolve().parents[1]


def clearance(x, y, obstacles, fires, config):
    values = [o.distance(x, y)-o.extra-config['robot_radius'] for o in obstacles]
    values += [math.hypot(x-fx, y-fy)-config['fire_radius']-config['robot_radius']
               for _, fx, fy in fires]
    return min(values)


def structural_check():
    config_path = PACKAGE/'step1/config.json'
    config = json.loads(config_path.read_text())
    route = json.loads((PACKAGE/'step1/route.json').read_text())
    grid = GridMap.load(PACKAGE/'step1/map.json')
    plan = grid
    world_file = PACKAGE/'worlds/warehouse.sdf'
    assert route['world_sha256'] == hashlib.sha256(world_file.read_bytes()).hexdigest(), 'World changed: regenerate the route'
    assert route['config_sha256'] == hashlib.sha256(config_path.read_bytes()).hexdigest(), 'Config changed: regenerate the route'
    assert 0 < config['linear_speed'] <= MAX_LINEAR_SPEED
    assert 0 < config['angular_speed'] <= MAX_ANGULAR_SPEED
    assert 0 < config['waypoint_approach_gain'] <= 5.0
    assert 0 < config['linear_acceleration'] <= 3.0
    assert 0 < config['linear_deceleration'] <= 3.0
    assert config['braking_response_seconds'] >= 0.12
    assert 0 < config['pose_max_age']+.06 <= config['braking_response_seconds']
    floors, obstacles, fires = read_geometry(world_file, config)
    assert read_geometry(PACKAGE/'worlds/warehouse_lite.sdf', config) == (floors, obstacles, fires), 'Lite world has different obstacles'
    expected = aisle_segments(plan, plan.reachable(plan.index(*config['start'][:2])), config)
    assert [s['name'] for s in route['aisle_segments']] and \
        Counter(s['name'] for s in expected) == Counter(s['name'] for s in route['aisle_segments']), 'Missing or repeated coverage sections'
    points = [(w['x'], w['y']) for w in route['waypoints']]
    assert math.dist(points[0], config['start'][:2]) < 1e-6
    assert math.dist(points[-1], config['start'][:2]) < 1e-6
    assert points[0][1] < -12.0 and max(y for _, y in points) > 10.0
    closest = float('inf')
    for a, b in zip(points, points[1:]):
        assert grid.line_safe(a, b), ('Unsafe route segment', a, b)
        count = max(1, int(math.ceil(math.dist(a, b)/0.05)))
        for i in range(count+1):
            x, y = a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count
            closest = min(closest, clearance(x, y, obstacles, fires, config))
            for angle in range(0, 360, 30):
                px = x+config['robot_radius']*math.cos(math.radians(angle))
                py = y+config['robot_radius']*math.sin(math.radians(angle))
                assert any(f.contains(px, py) for f in floors), 'Robot footprint goes off the floor'
    assert closest >= config['safety_margin'], ('Insufficient route clearance', closest)
    robot = ET.parse(PACKAGE/'models/aisle_robot/model.sdf').getroot().find('model')
    assert robot.findtext('static') == 'false'
    links = {link.get('name'): link for link in robot.findall('link')}
    for link in links.values():
        assert float(link.findtext('inertial/mass')) > 0
        for tag in ('ixx', 'iyy', 'izz'):
            assert float(link.findtext('inertial/inertia/'+tag)) > 0
    joints = {j.get('name'): j for j in robot.findall('joint')}
    drive = next(p for p in robot.findall('plugin') if p.get('name').endswith('::DiffDrive'))
    assert float(drive.findtext('max_linear_velocity')) >= config['linear_speed']
    assert float(drive.findtext('max_linear_acceleration')) == config['linear_acceleration']
    assert -float(drive.findtext('min_linear_acceleration')) == config['linear_deceleration']
    pose_publisher = next(p for p in robot.findall('plugin') if p.get('name').endswith('::PosePublisher'))
    assert float(pose_publisher.findtext('update_frequency')) >= 100
    for side in ('left', 'right'):
        joint = joints[drive.findtext(side+'_joint')]
        assert joint.get('type') == 'revolute'
        assert joint.findtext('axis/xyz').strip() == '0 1 0'
        link = links[joint.findtext('child')]
        z = float(link.findtext('pose').split()[2])
        radius = float(link.findtext('collision/geometry/cylinder/radius'))
        width = float(link.findtext('collision/geometry/cylinder/length'))
        pose = list(map(float, link.findtext('pose').split()))
        assert math.hypot(abs(pose[0])+radius, abs(pose[1])+width/2) < config['robot_radius'], \
            'Wheel footprint exceeds the planned radius'
        assert abs(z-radius) < 1e-8, 'Drive wheel is not at floor height'
        assert abs(radius-float(drive.findtext('wheel_radius'))) < 1e-8
        worst_wheel_speed = (config['linear_speed'] + \
            float(drive.findtext('wheel_separation'))*config['angular_speed']/2)/radius
        assert worst_wheel_speed <= float(joint.findtext('axis/limit/velocity'))
    left_y = float(links['left_wheel'].findtext('pose').split()[1])
    right_y = float(links['right_wheel'].findtext('pose').split()[1])
    assert abs(left_y-right_y-float(drive.findtext('wheel_separation'))) < 1e-8
    # Conservative bounding circle about the axle covers every chassis corner.
    chassis_pose = list(map(float, links['chassis'].findtext('pose').split()))
    chassis_size = list(map(float, links['chassis'].findtext('collision/geometry/box/size').split()))
    assert chassis_pose[2]+chassis_size[2]/2+.005 <= config['robot_height'], \
        'Chassis is higher than the obstacle map allows'
    for sx in (-1, 1):
        for sy in (-1, 1):
            assert math.hypot(chassis_pose[0]+sx*chassis_size[0]/2,
                              chassis_pose[1]+sy*chassis_size[1]/2) < config['robot_radius']
    support_pose = list(map(float, links['support_ball'].findtext('pose').split()))
    support_radius = float(links['support_ball'].findtext('collision/geometry/sphere/radius'))
    assert math.hypot(*support_pose[:2])+support_radius < config['robot_radius']
    for stem in ('warehouse_step1', 'warehouse_step1_lite'):
        world = ET.parse(PACKAGE/f'worlds/{stem}.sdf').getroot().find('world')
        assert not world.findall('include'), 'Demo world needs an external model lookup'
        robots = [m for m in world.findall('model') if m.get('name') == config['model_name']]
        assert len(robots) == 1 and robots[0].findtext('static') == 'false'
        spawn = list(map(float, robots[0].findtext('pose').split()))
        assert spawn == [config['start'][0], config['start'][1], .005, 0, 0, config['start'][2]]
        # The generated world must contain exactly the current robot definition.
        def normalized(element):
            return (element.tag, tuple(sorted(element.attrib.items())),
                    (element.text or '').strip(), tuple(normalized(child) for child in element))
        for tag in ('link', 'joint', 'plugin'):
            assert [normalized(e) for e in robots[0].findall(tag)] == \
                [normalized(e) for e in robot.findall(tag)], 'Robot world is stale: run plan'
        source = ET.parse(PACKAGE/'worlds'/('warehouse_lite.sdf' if stem.endswith('_lite') else 'warehouse.sdf')).getroot().find('world')
        environment = [m for m in world.findall('model') if m.get('name') != config['model_name']]
        assert [ET.tostring(m) for m in environment] == [ET.tostring(m) for m in source.findall('model')], 'Environment geometry was modified'
    return config, route, grid, floors, obstacles, fires, closest


def numerical_run(label, dt, lag, slip, noise, config, route, grid, floors, obstacles, fires):
    follower = Follower(route['waypoints'], config, grid)
    x, y, yaw = config['start']
    left, right, rng = 0.0, 0.0, random.Random(4)
    robot = ET.parse(PACKAGE/'models/aisle_robot/model.sdf').getroot().find('model')
    drive = next(p for p in robot.findall('plugin') if p.get('name').endswith('::DiffDrive'))
    separation = float(drive.findtext('wheel_separation'))
    minimum, sections = float('inf'), set()
    peak_command, peak_actual, limited_linear = 0.0, 0.0, 0.0
    for i in range(int(1800/dt)):
        previous_index = follower.index
        v, w = follower.command(x+rng.uniform(-noise, noise), y+rng.uniform(-noise, noise),
                                yaw+rng.uniform(-noise, noise), dt)
        peak_command = max(peak_command, v)
        # Mirror the configured DiffDrive acceleration limiter before motor lag.
        limited_linear = max(limited_linear-config['linear_deceleration']*dt,
                             min(v, limited_linear+config['linear_acceleration']*dt))
        if follower.index != previous_index:
            for k in range(previous_index, min(follower.index, len(route['waypoints']))):
                waypoint = route['waypoints'][k]
                if waypoint['kind'] == 'aisle':
                    sections.add(waypoint['name'])
        alpha = 1-math.exp(-dt/lag) if lag else 1.0
        left += (limited_linear-separation*w/2-left)*alpha
        right += (limited_linear+separation*w/2-right)*alpha
        # Deliberately apply unequal wheel travel to test correction from model pose.
        lv, rv = left*(1+slip), right*(1-slip)
        linear, angular = (lv+rv)/2, (rv-lv)/separation
        peak_actual = max(peak_actual, linear)
        previous_xy = (x, y)
        if abs(angular) < 1e-10:
            x += linear*dt*math.cos(yaw)
            y += linear*dt*math.sin(yaw)
        else:
            x += linear/angular*(math.sin(yaw+angular*dt)-math.sin(yaw))
            y -= linear/angular*(math.cos(yaw+angular*dt)-math.cos(yaw))
        yaw += angular*dt
        assert grid.line_safe(previous_xy, (x, y)), (label, 'Motion crossed a blocked map cell',
                follower.index, previous_xy, (x, y), v, w, linear, angular, follower.mode)
        assert grid.safe(x, y), (label, 'Actual centre left the safe map', follower.index, x, y)
        if i % max(1, int(0.20/dt)) == 0:
            gap = clearance(x, y, obstacles, fires, config)
            minimum = min(minimum, gap)
            assert gap >= config['safety_margin'], (label, 'Insufficient physical clearance', gap)
        if follower.finished:
            expected = {s['name'] for s in route['aisle_segments']}
            assert sections == expected, (label, 'Aisle coverage incomplete')
            error = math.dist((x, y), config['start'][:2])
            assert error <= config['waypoint_tolerance']+noise*2
            v, w = follower.command(x, y, yaw, dt)
            assert v == w == 0.0, 'Finished controller does not command a stop'
            return {'name': label, 'passed': True, 'simulation_seconds': round(i*dt, 2),
                    'nominal_demo_seconds_at_requested_rate':
                        round((i*dt+config['start_delay'])/config['demo_time_factor'], 2),
                    'finish_error_metres': round(error, 4),
                    'peak_command_metres_per_second': round(peak_command, 3),
                    'peak_numerical_speed_metres_per_second': round(peak_actual, 3),
                    'minimum_clearance_beyond_robot_radius_metres': round(minimum, 4),
                    'aisle_sections_visited': len(sections)}
    raise AssertionError(label+': tour exceeded 30 simulated minutes')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--structural-only', action='store_true')
    args = parser.parse_args()
    config, route, grid, floors, obstacles, fires, closest = structural_check()
    print(f'PASS: geometry unchanged, route safe, {len(route["aisle_segments"])} aisle sections, start/finish outside.')
    if args.structural_only:
        return
    cases = [('ideal_50Hz', .02, 0.0, 0.0, 0.0),
             ('motor_response_60ms', 1/30, .06, 0.0, 0.0),
             ('wheel_difference_2percent_pose_noise_5mm', 1/30, .06, .02, .005),
             ('feedback_20Hz_with_slip_and_noise', .05, .06, .02, .005)]
    results = []
    for case in cases:
        result = numerical_run(*case, config, route, grid, floors, obstacles, fires)
        results.append(result)
        print(f'PASS: {result["name"]}, finish error {result["finish_error_metres"]} m, '
              f'{result["aisle_sections_visited"]} sections visited, '
              f'{result["simulation_seconds"]:g} sim seconds, '
              f'peak numerical speed {result["peak_numerical_speed_metres_per_second"]:g} m/s.')
    report = {'offline_checks_passed': True, 'gazebo_physics_tested_here': False,
              'gazebo_rendering_tested_here': False,
              'planned_distance_metres': route['distance_metres'],
              'driving_speed_metres_per_second': config['linear_speed'],
              'requested_demo_time_factor': config['demo_time_factor'],
              'video_target_seconds': config['video_target_seconds'],
              'planned_minimum_clearance_beyond_robot_radius_metres': round(closest, 4),
              'cases': results,
              'note': 'Numerical differential-drive tests do not replace a Gazebo contact-physics run.'}
    (PACKAGE/'step1/offline_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Offline validation complete. Gazebo contact physics and rendering require a runtime run.')


if __name__ == '__main__':
    main()
