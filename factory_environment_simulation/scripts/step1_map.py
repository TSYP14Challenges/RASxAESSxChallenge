#!/usr/bin/env python3
"""Known-world geometry and a conservative grid map; Python standard library only."""
from collections import deque
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def pose_matrix(text):
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                             math.cos(pitch), math.sin(pitch),
                             math.cos(yaw), math.sin(yaw))
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, x],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, y],
            [-sp, cp*sr, cp*cr, z], [0, 0, 0, 1]]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4))
             for j in range(4)] for i in range(4)]


def box_bounds(transform, size):
    """Project all eight transformed corners, including roll/pitch of fallen boxes."""
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                v = [sx*size[0]/2, sy*size[1]/2, sz*size[2]/2, 1]
                corners.append([sum(transform[i][k]*v[k] for k in range(4))
                                for i in range(3)])
    return [min(v[i] for v in corners) for i in range(3)] + \
           [max(v[i] for v in corners) for i in range(3)]


@dataclass
class Rectangle:
    name: str
    bounds: list
    extra: float = 0.0

    def distance(self, x, y):
        x0, y0, x1, y1 = self.bounds
        return math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1))

    def contains(self, x, y):
        x0, y0, x1, y1 = self.bounds
        return x0 <= x <= x1 and y0 <= y <= y1


def read_geometry(world_file, config):
    world = ET.parse(world_file).getroot().find('world')
    floors, obstacles, fires, pits = [], [], [], []
    for model in world.findall('model'):
        name = model.get('name')
        model_tf = pose_matrix(model.findtext('pose'))
        projected = []
        for link in model.findall('link'):
            link_tf = multiply(model_tf, pose_matrix(link.findtext('pose')))
            for collision in link.findall('collision'):
                size_text = collision.findtext('geometry/box/size')
                if size_text is None:
                    raise ValueError('Unexpected collision geometry in ' + name)
                tf = multiply(link_tf, pose_matrix(collision.findtext('pose')))
                bounds = box_bounds(tf, list(map(float, size_text.split())))
                xy = [bounds[0], bounds[1], bounds[3], bounds[4]]
                if name in ('unified_factory_floor', 'entrance_apron'):
                    if abs(bounds[5]) > 0.02:
                        raise ValueError('Unexpected floor height')
                    floors.append(Rectangle(name, xy))
                elif bounds[2] <= config['robot_height'] and bounds[5] >= 0.001:
                    projected.append(xy)
        if name.startswith('rack_') and projected:
            # Treat the entire rack footprint as solid, rather than weaving under it.
            projected = [[min(b[0] for b in projected), min(b[1] for b in projected),
                          max(b[2] for b in projected), max(b[3] for b in projected)]]
        extra = config['victim_extra_margin'] if name.startswith('victim_') else 0.0
        obstacles.extend(Rectangle(name, xy, extra) for xy in projected)
        if name.startswith('fire_'):
            fires.append((name, model_tf[0][3], model_tf[1][3]))
        if name.startswith('pit_'):
            # The bottom collision is below the floor; explicitly reserve the opening.
            x, y = model_tf[0][3], model_tf[1][3]
            pits.append(Rectangle(name, [x-1.2, y-1.2, x+1.2, y+1.2],
                                  config['pit_extra_margin']))
    if not floors or len(pits) != 2 or len(fires) != 6:
        raise ValueError('World does not match the supplied factory scene')
    return floors, obstacles + pits, fires


class GridMap:
    def __init__(self, resolution, origin, width, height, free):
        self.resolution, self.origin = resolution, origin
        self.width, self.height, self.free = width, height, bytearray(free)

    def index(self, x, y):
        return int(math.floor((y-self.origin[1])/self.resolution+0.5)), \
               int(math.floor((x-self.origin[0])/self.resolution+0.5))

    def point(self, cell):
        row, col = cell
        return (self.origin[0]+col*self.resolution, self.origin[1]+row*self.resolution)

    def is_free(self, cell):
        row, col = cell
        return 0 <= row < self.height and 0 <= col < self.width and \
            bool(self.free[row*self.width+col])

    def safe(self, x, y):
        return self.is_free(self.index(x, y))

    def line_safe(self, a, b, buffer=0.0):
        n = max(1, int(math.ceil(math.dist(a, b)/(self.resolution/5))))
        shifts = [(0, 0)]
        if buffer:
            shifts += [(buffer*math.cos(i*math.pi/4), buffer*math.sin(i*math.pi/4))
                       for i in range(8)]
        return all(self.safe(a[0]+dx+(b[0]-a[0])*i/n, a[1]+dy+(b[1]-a[1])*i/n)
                   for dx, dy in shifts for i in range(n+1))

    def neighbours(self, cell):
        row, col = cell
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = row+dr, col+dc
            if not self.is_free(nxt):
                continue
            if dr and dc and not (self.is_free((row+dr, col)) and
                                  self.is_free((row, col+dc))):
                continue  # No diagonal corner cutting.
            yield nxt, math.hypot(dr, dc)

    def reachable(self, start):
        if not self.is_free(start):
            raise ValueError('Start position is blocked or unsupported')
        seen, queue = {start}, deque([start])
        while queue:
            for nxt, _ in self.neighbours(queue.popleft()):
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return seen

    def shortest_path(self, start, goal):
        if not self.is_free(start) or not self.is_free(goal):
            raise ValueError('A* endpoint is not safe')
        queue, cost, parent = [(math.dist(start, goal), 0.0, start)], {start: 0.0}, {}
        while queue:
            _, g, cell = heapq.heappop(queue)
            if g > cost[cell]+1e-9:
                continue
            if cell == goal:
                path = [cell]
                while cell != start:
                    cell = parent[cell]
                    path.append(cell)
                return list(reversed(path))
            for nxt, step in self.neighbours(cell):
                new = g+step
                if new < cost.get(nxt, float('inf')):
                    cost[nxt], parent[nxt] = new, cell
                    heapq.heappush(queue, (new+math.dist(nxt, goal), new, nxt))
        raise ValueError('No safe connection between aisle segments')

    def simplify(self, points, tracking_buffer=0.0):
        if len(points) < 3:
            return points
        result, i = [points[0]], 0
        while i < len(points)-1:
            # Advance greedily while every straight segment remains collision-free.
            j = i+1
            while j+1 < len(points) and self.line_safe(points[i], points[j+1], tracking_buffer):
                j += 1
            result.append(points[j])
            i = j
        return result

    def save(self, path):
        rows = [''.join('1' if x else '0' for x in self.free[r*self.width:(r+1)*self.width])
                for r in range(self.height)]
        Path(path).write_text(json.dumps({'resolution': self.resolution,
                                         'origin': self.origin, 'rows': rows})+'\n')

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        rows = data['rows']
        return cls(data['resolution'], data['origin'], len(rows[0]), len(rows),
                   [int(x) for row in rows for x in row])


def build_grid(floors, obstacles, fires, config):
    res = config['grid_resolution']
    x0 = math.floor(min(r.bounds[0] for r in floors)/res)*res-res
    y0 = math.floor(min(r.bounds[1] for r in floors)/res)*res-res
    width = int(round((max(r.bounds[2] for r in floors)-x0)/res))+2
    height = int(round((max(r.bounds[3] for r in floors)-y0)/res))+2
    radius = config['robot_radius']+config['safety_margin']
    # Half a cell diagonal conservatively covers continuous positions in a cell.
    pad = res/math.sqrt(2)
    grid = GridMap(res, [x0, y0], width, height, [0]*(width*height))
    supported = bytearray(width*height)
    for row in range(height):
        for col in range(width):
            x, y = grid.point((row, col))
            supported[row*width+col] = any(r.contains(x, y) for r in floors)
    n = int(math.ceil((radius+pad)/res))
    offsets = [(dr, dc) for dr in range(-n, n+1) for dc in range(-n, n+1)
               if math.hypot(dr*res, dc*res) <= radius+pad]
    for row in range(height):
        for col in range(width):
            if not supported[row*width+col]:
                continue
            if all(0 <= row+dr < height and 0 <= col+dc < width and
                   supported[(row+dr)*width+col+dc] for dr, dc in offsets):
                grid.free[row*width+col] = 1
    for obstacle in obstacles:
        limit = radius+pad+obstacle.extra
        bx0, by0, bx1, by1 = obstacle.bounds
        r0, c0 = grid.index(bx0-limit-res, by0-limit-res)
        r1, c1 = grid.index(bx1+limit+res, by1+limit+res)
        for row in range(max(0, r0), min(height-1, r1)+1):
            for col in range(max(0, c0), min(width-1, c1)+1):
                if obstacle.distance(*grid.point((row, col))) <= limit:
                    grid.free[row*width+col] = 0
    for _, fx, fy in fires:
        limit = config['fire_radius']+radius+pad
        r0, c0 = grid.index(fx-limit-res, fy-limit-res)
        r1, c1 = grid.index(fx+limit+res, fy+limit+res)
        for row in range(max(0, r0), min(height-1, r1)+1):
            for col in range(max(0, c0), min(width-1, c1)+1):
                if math.dist(grid.point((row, col)), (fx, fy)) <= limit:
                    grid.free[row*width+col] = 0
    return grid
