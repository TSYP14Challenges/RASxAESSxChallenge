#!/usr/bin/env python3
"""Extend the working scene; keep every original environment model and aisle."""
from collections import defaultdict
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from step1_map import build_grid, read_geometry
from step2_events import COLORS, PACKAGE, event_gap, load_config, read_events
from step2_timeline import MissionTimeline
from step2_network import wire_vertices


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def target_approach(event, grid, reachable, originals):
    cx, cy = event['centre']
    candidates = []
    for row, col in reachable:
        x, y = grid.point((row, col))
        if abs(x-cx) > 2.0 or abs(y-cy) > 2.0:
            continue
        yaw = math.atan2(cy-y, cx-x)-math.pi/2
        gap = event_gap({'x': x, 'y': y, 'yaw': yaw}, event)
        if .12 <= gap <= event['distance']-.035:
            anchor = min(range(1, len(originals)-1), key=lambda i:
                         math.dist((x, y), (originals[i]['x'], originals[i]['y'])))
            score = math.dist((x, y), (originals[anchor]['x'], originals[anchor]['y']))
            candidates.append((score, abs(gap-.22), anchor, (row, col), yaw))
    if not candidates:
        raise RuntimeError('No safe 30 cm approach for '+event['id'])
    _, _, anchor, goal, yaw = min(candidates)
    a = originals[anchor]
    path = grid.shortest_path(grid.index(a['x'], a['y']), goal)
    points = grid.simplify([grid.point(p) for p in path])
    points[0] = a['x'], a['y']
    excursion = [{'x': x, 'y': y, 'kind': 'connection',
                  'name': 'Approach '+event['id']} for x, y in points[1:]]
    excursion[-1].update(kind='event_approach', face_yaw=yaw, event_id=event['id'])
    excursion += [{'x': x, 'y': y, 'kind': 'connection',
                   'name': 'Resume aisle tour'} for x, y in reversed(points[:-1])]
    return anchor, excursion


def material(parent, color, emission=.08):
    m = ET.SubElement(parent, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(m, tag).text = ' '.join(map(str, color))
    ET.SubElement(m, 'emissive').text = ' '.join(map(str, [v*emission for v in color[:3]]+[1]))


def box_visual(link, name, size, color, pose='0 0 0 0 0 0', emission=.08):
    v = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(v, 'pose').text = pose
    ET.SubElement(ET.SubElement(v, 'geometry'), 'box').append(ET.Element('size'))
    v.find('geometry/box/size').text = ' '.join(map(str, size))
    material(v, color, emission)
    return v


def floating_model(name, position, publisher=True, fixed=False):
    m = ET.Element('model', name=name)
    ET.SubElement(m, 'static').text = 'true' if fixed else 'false'
    ET.SubElement(m, 'allow_auto_disable').text = 'false'
    ET.SubElement(m, 'pose').text = ' '.join(map(str, list(position)+[0, 0, 0]))
    l = ET.SubElement(m, 'link', name='body')
    ET.SubElement(l, 'gravity').text = 'false'
    inertial = ET.SubElement(l, 'inertial')
    ET.SubElement(inertial, 'mass').text = '0.03'
    inertia = ET.SubElement(inertial, 'inertia')
    for tag, value in [('ixx', '.000016'), ('iyy', '.000016'), ('izz', '.000032'),
                       ('ixy', '0'), ('ixz', '0'), ('iyz', '0')]:
        ET.SubElement(inertia, tag).text = value
    if publisher:
        plugin = ET.SubElement(m, 'plugin', filename='gz-sim-pose-publisher-system',
                               name='gz::sim::systems::PosePublisher')
        for tag, value in [('publish_link_pose', 'false'), ('publish_visual_pose', 'false'),
                           ('publish_collision_pose', 'false'), ('publish_sensor_pose', 'false'),
                           ('publish_model_pose', 'true'), ('publish_nested_model_pose', 'true'),
                           ('static_publisher', 'false'), ('use_pose_vector_msg', 'true'),
                           ('update_frequency', '10')]:
            ET.SubElement(plugin, tag).text = value
        if fixed:
            # Static poses stay on this small periodic stream because
            # static_publisher is false. No full-scene subscription is needed.
            ET.SubElement(plugin, 'topic').text = f'/model/{name}/pose'
    return m, l


def beacon_specs(events, config):
    specs = [('step2_entrance_blue_beacon', 'gateway')]
    specs += [('step2_'+e['id']+'_beacon', e['kind']) for e in events]
    specs += [(f'step2_range_beacon_{i:02d}', 'range')
              for i in range(1, config['range_beacon_capacity']+1)]
    return specs


def beacon_sdf(name, kind, position, config):
    """Create a tile definition at its real drop position, never below the floor."""
    if not all(math.isfinite(v) for v in position) or \
            position[2]-config['beacon_size_metres'][2]/2 <= 0:
        raise ValueError('Beacon must be placed above the floor')
    gateway = kind == 'gateway'
    model, link = floating_model(name, position, fixed=gateway)
    box_visual(link, 'tile', config['beacon_size_metres'], COLORS[kind],
               emission=.65 if gateway else .08)
    if gateway:
        add_gateway_wire(link, config, position)
        for visual in link.findall('visual'):
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            ET.SubElement(visual, 'transparency').text = '0'
            ET.SubElement(visual, 'visibility_flags').text = '4294967295'
    root = ET.Element('sdf', version='1.9')
    root.append(model)
    return ET.tostring(root, encoding='unicode')


def add_gateway_wire(link, config, origin):
    """Create wire visuals in the tile's local frame, in the same model."""
    gx, gy, gz = origin
    vertices = wire_vertices(config)
    for i, (a, b) in enumerate(zip(vertices, vertices[1:])):
        yaw = math.atan2(b[1]-a[1], b[0]-a[0])
        box_visual(link, 'cable_'+str(i), [math.dist(a, b), .04, .025], [.025, .03, .04, 1],
                   f'{(a[0]+b[0])/2-gx} {(a[1]+b[1])/2-gy} {(a[2]+b[2])/2-gz} 0 0 {yaw}')


def make_world(events, config):
    root = ET.parse(PACKAGE/'worlds/warehouse_step1_preview.sdf').getroot()
    world = root.find('world')
    # The entrance assembly is part of the scene from startup. Other tiles
    # reserve subscriptions here and are created later at their event drops.
    models = [name for name, _ in beacon_specs(events, config)]
    network = ET.SubElement(world, 'model', name='step2_outside_gateway')
    ET.SubElement(network, 'static').text = 'true'
    ET.SubElement(network, 'pose').text = ' '.join(map(str, config['outside_gateway_position']+[0, 0, 0]))
    link = ET.SubElement(network, 'link', name='body')
    visual = box_visual(link, 'blue_box', [.45, .35, .45], COLORS['gateway'])
    c = ET.SubElement(link, 'collision', name='box_collision')
    c.append(deepcopy(visual.find('geometry')))
    world.append(ET.fromstring(beacon_sdf('step2_entrance_blue_beacon', 'gateway',
                 config['entrance_beacon_position'], config)).find('model'))
    ET.indent(root)
    ET.ElementTree(root).write(PACKAGE/'worlds/warehouse_step2.sdf',
                               encoding='utf-8', xml_declaration=True)
    return models


def main():
    config, originals = load_config(), json.loads((PACKAGE/'step1/route.json').read_text())
    events = read_events(config)
    floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    grid = build_grid(floors, obstacles, fires, config)
    reachable = grid.reachable(grid.index(*config['start'][:2]))
    additions = defaultdict(list)
    for event in events:
        if event['kind'] == 'victim':
            anchor, excursion = target_approach(event, grid, reachable, originals['waypoints'])
            additions[anchor] += excursion
    points = []
    inserted_gateway = False
    for i, w in enumerate(originals['waypoints']):
        if i and not inserted_gateway:
            a = originals['waypoints'][i-1]
            gx, gy = config['entrance_robot_stop']
            if a['y'] < gy <= w['y'] and abs(a['x']) < 1e-7 and abs(w['x']) < 1e-7:
                points.append({'x': gx, 'y': gy, 'kind': 'gateway_stop',
                               'name': 'Entrance gateway beacon'})
                inserted_gateway = True
        points.append(deepcopy(w))
        points.extend(additions[i])
    if not inserted_gateway:
        raise RuntimeError('Could not add the entrance gateway stop')
    if not all(grid.line_safe((a['x'], a['y']), (b['x'], b['y']))
               for a, b in zip(points, points[1:])):
        raise RuntimeError('Extended route crosses an obstacle')
    timeline = MissionTimeline(points, config, config['motion_seconds'])
    data = {'world_sha256': digest(PACKAGE/'worlds/warehouse.sdf'),
            'step1_route_sha256': digest(PACKAGE/'step1/route.json'),
            'config_sha256': digest(PACKAGE/'step2/config.json'),
            'step1_config_sha256': digest(PACKAGE/'step1/config.json'),
            'aisle_segments': originals['aisle_segments'], 'waypoints': points,
            'events': events, 'pose_models': make_world(events, config),
            'beacon_deployment': 'fixed_entrance_and_create_event_tiles',
            'distance_metres': round(timeline.distance, 3)}
    grid.save(PACKAGE/'step2/map.json')
    (PACKAGE/'step2/route.json').write_text(json.dumps(data, indent=2)+'\n')
    (PACKAGE/'step2/entrance_blue_beacon.sdf').write_text(
        beacon_sdf('step2_entrance_blue_beacon', 'gateway', config['entrance_beacon_position'], config)+'\n')
    # Older packages used a separate, world-origin cable model. The wire is
    # now part of the fixed entrance model already included in the world.
    (PACKAGE/'step2/gateway_cable.sdf').unlink(missing_ok=True)
    (PACKAGE/'step2/gateway_beacon.sdf').unlink(missing_ok=True)
    print(f'Built step 2: {sum(e["kind"]=="fire" for e in events)} fires, '
          f'{sum(e["kind"]=="victim" for e in events)} victims, 15 aisle sections, '
          f'{timeline.distance:.1f} m; adjustable {config["communication_range_metres"]:g} m beacon range.')


if __name__ == '__main__':
    main()
