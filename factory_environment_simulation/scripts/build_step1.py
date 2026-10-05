#!/usr/bin/env python3
"""Create the known-layout aisle tour, robot worlds, and an exact route preview."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from step1_map import build_grid, read_geometry

PACKAGE = Path(__file__).resolve().parents[1]


def aisle_segments(grid, reachable, config):
    """One coverage task per reachable uninterrupted portion of an aisle centreline."""
    segments = []
    for axis, coords in (('horizontal', config['horizontal_aisles']),
                         ('vertical', config['vertical_aisles'])):
        for value in coords:
            if axis == 'horizontal':
                row = grid.index(0, value)[0]
                cells = [(row, col) for col in range(grid.width)]
            else:
                col = grid.index(value, 0)[1]
                cells = [(row, col) for row in range(grid.height)]
            run = []
            for cell in cells + [None]:
                point = grid.point(cell) if cell is not None else (0, -99)
                usable = cell in reachable and -11.4 <= point[1] <= 11.4
                if usable:
                    run.append(cell)
                    continue
                if len(run) >= 3:
                    segments.append({'name': f'{axis} {value:g} / section {len(segments)+1}',
                                     'cells': run})
                run = []
    return segments


def create_route(grid, segments, start, tracking_buffer=0.055):
    current, pending, route, visited = start, list(segments), [], []
    all_cells = [start]
    while pending:
        options = []
        for i, segment in enumerate(pending):
            for reverse in (False, True):
                cells = list(reversed(segment['cells'])) if reverse else segment['cells']
                options.append((math.dist(current, cells[0]), i, reverse))
        # Evaluate a few geometrically near candidates with the actual safe A* distance.
        best = None
        for _, i, reverse in sorted(options)[:8]:
            segment = pending[i]
            cells = list(reversed(segment['cells'])) if reverse else segment['cells']
            connection = grid.shortest_path(current, cells[0])
            length = sum(math.dist(a, b) for a, b in zip(connection, connection[1:]))
            candidate = (length, i, reverse, connection, cells)
            if best is None or length < best[0]:
                best = candidate
        _, i, reverse, connection, cells = best
        segment = pending.pop(i)
        points = grid.simplify([grid.point(c) for c in connection], tracking_buffer)
        if len(points) > 1:
            route.append({'name': 'Connect to ' + segment['name'],
                          'kind': 'connection', 'points': points})
        route.append({'name': segment['name'], 'kind': 'aisle',
                      'points': [grid.point(cells[0]), grid.point(cells[-1])]})
        visited.append({'name': segment['name'], 'start': grid.point(cells[0]),
                        'end': grid.point(cells[-1]), 'samples': len(cells)})
        all_cells.extend(connection[1:])
        all_cells.extend(cells[1:])
        current = cells[-1]
    home = grid.shortest_path(current, start)
    route.append({'name': 'Return outside and stop', 'kind': 'return',
                  'points': grid.simplify([grid.point(c) for c in home], tracking_buffer)})
    all_cells.extend(home[1:])
    waypoints = [{'x': grid.point(start)[0], 'y': grid.point(start)[1],
                  'name': 'Start outside', 'kind': 'start'}]
    for section in route:
        for point in section['points']:
            if math.dist(point, (waypoints[-1]['x'], waypoints[-1]['y'])) < 0.01:
                continue
            waypoints.append({'x': round(point[0], 4), 'y': round(point[1], 4),
                              'name': section['name'], 'kind': section['kind']})
    length = sum(math.dist((a['x'], a['y']), (b['x'], b['y']))
                 for a, b in zip(waypoints, waypoints[1:]))
    return waypoints, visited, length


def write_preview(path, floors, obstacles, fires, waypoints, config):
    """Vector preview, accurate to world coordinates; no graphics dependencies."""
    scale = 23
    def point(x, y):
        return (round((x+21)*scale, 2), round((13-y)*scale, 2))
    def rect(bounds, color):
        x0, y0, x1, y1 = bounds
        x, y = point(x0, y1)
        return f'<rect x="{x}" y="{y}" width="{(x1-x0)*scale:.2f}" height="{(y1-y0)*scale:.2f}" fill="{color}"/>'
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="760" viewBox="0 0 1080 760">',
                '<rect width="1080" height="760" fill="#f3f6fa"/>',
                '<g transform="translate(38 14)">']
    elements += [rect(f.bounds, '#e5e9ed') for f in floors]
    for obstacle in obstacles:
        color = '#555d67'
        if obstacle.name.startswith('rack_'):
            color = '#af8051'
        elif obstacle.name.startswith('victim_'):
            color = '#8660a8'
        elif obstacle.name.startswith('pit_'):
            color = '#151b25'
        elements.append(rect(obstacle.bounds, color))
    for _, x, y in fires:
        sx, sy = point(x, y)
        elements.append(f'<circle cx="{sx}" cy="{sy}" r="{config["fire_radius"]*scale}" fill="#ed6a2c" fill-opacity=".4"/>')
    line = ' '.join(f'{point(w["x"], w["y"])[0]},{point(w["x"], w["y"])[1]}' for w in waypoints)
    elements.append(f'<polyline points="{line}" fill="none" stroke="#07899a" stroke-width="2" stroke-opacity=".85"/>')
    x, y = point(*config['start'][:2])
    elements.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#008e60" stroke="white" stroke-width="2"/>')
    elements.append(f'<text x="{x+13}" y="{y+4}" font-family="sans-serif" font-size="15">Start / finish</text>')
    elements += ['</g>', '<text x="42" y="724" font-family="sans-serif" font-size="20" fill="#243343">Step 1: known-layout aisle tour</text>',
                 '<text x="42" y="750" font-family="sans-serif" font-size="14" fill="#4c5c6f">Teal: safe planned route | Brown: racks | Orange: fires | Black: pits | Purple: people</text>', '</svg>']
    path.write_text('\n'.join(elements)+'\n')


def robot_world(source, target, config):
    root = ET.parse(source).getroot()
    world = root.find('world')
    # Inline the robot so `gz sdf -k` needs no Gazebo URI callback to load it.
    robot = ET.parse(PACKAGE/'models/aisle_robot/model.sdf').getroot().find('model')
    robot.set('name', config['model_name'])
    x, y, yaw = config['start']
    robot.insert(0, ET.Element('pose'))
    robot.find('pose').text = f'{x} {y} 0.005 0 0 {yaw}'
    world.append(robot)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)


def main():
    config = json.loads((PACKAGE/'step1/config.json').read_text())
    world_file = PACKAGE/'worlds/warehouse.sdf'
    floors, obstacles, fires = read_geometry(world_file, config)
    grid = build_grid(floors, obstacles, fires, config)
    start = grid.index(*config['start'][:2])
    reachable = grid.reachable(start)
    segments = aisle_segments(grid, reachable, config)
    waypoints, visits, length = create_route(grid, segments, start, config['segment_tracking_buffer'])
    route = {'world_sha256': hashlib.sha256(world_file.read_bytes()).hexdigest(),
             'config_sha256': hashlib.sha256((PACKAGE/'step1/config.json').read_bytes()).hexdigest(),
             'distance_metres': round(length, 2), 'aisle_segments': visits,
             'reachable_cells': len(reachable), 'waypoints': waypoints}
    grid.save(PACKAGE/'step1/map.json')
    (PACKAGE/'step1/route.json').write_text(json.dumps(route, indent=2)+'\n')
    write_preview(PACKAGE/'step1/route_preview.svg', floors, obstacles, fires, waypoints, config)
    robot_world(world_file, PACKAGE/'worlds/warehouse_step1.sdf', config)
    robot_world(PACKAGE/'worlds/warehouse_lite.sdf', PACKAGE/'worlds/warehouse_step1_lite.sdf', config)
    from build_step1_preview import build
    build(config)
    print(f'Planned {len(visits)} reachable aisle sections, {len(waypoints)} waypoints, {length:.1f} metres.')
    print(f'Straight-line travel time at {config["linear_speed"]} m/s: {length/config["linear_speed"]/60:.1f} min; allow time for turns.')
    print('Start and finish:', config['start'][:2])


if __name__ == '__main__':
    main()
