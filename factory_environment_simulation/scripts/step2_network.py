#!/usr/bin/env python3
"""Logical beacon messages: interior radio relays, then a required wired uplink.

This is a scenario communication model, not an RF or Ethernet physics model.
Only confirmed, deployed beacons can be registered. The outside gateway is
never a radio peer: every delivered message must finish through the entrance
blue beacon and its cable.
"""
from copy import deepcopy
import math

ENTRANCE_BEACON = 'step2_entrance_blue_beacon'
OUTSIDE_GATEWAY = 'step2_outside_gateway'


def wire_vertices(config):
    nx, ny, _ = config['outside_gateway_position']
    bx, by, _ = config['entrance_beacon_position']
    return [(nx, ny+.175, .020), (nx, by-.5, .020), (bx, by, .020)]


class BeaconNetwork:
    def __init__(self, config):
        self.config = config
        self.nodes, self.inbox, self.pending = {}, {}, {}
        self.cable_connected = False

    def register(self, record):
        name, parent = record['beacon_name'], record['parent_beacon']
        if name in self.nodes:
            raise RuntimeError('Beacon already registered: '+name)
        if name == OUTSIDE_GATEWAY:
            raise RuntimeError('The outside gateway cannot act as an interior radio beacon')
        position = list(record['position'])
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise RuntimeError('Invalid beacon position: '+name)
        if record['kind'] == 'gateway':
            if name != ENTRANCE_BEACON or parent is not None or self.nodes:
                raise RuntimeError('The entrance blue beacon must be the first interior node')
            if math.dist(position, self.config['entrance_beacon_position']) > 1e-8:
                raise RuntimeError('The blue beacon does not meet the entrance cable endpoint')
        else:
            if parent not in self.nodes:
                raise RuntimeError('No deployed interior relay for '+name)
            distance = math.dist(position[:2], self.nodes[parent]['position'][:2])
            if distance > self.config['communication_range_metres']+1e-8:
                raise RuntimeError('Interior radio link exceeds the beacon range')
        self.nodes[name] = {'position': position, 'parent': parent,
                            'role': 'entrance_uplink' if record['kind'] == 'gateway' else record['kind']}

    def connect_cable(self):
        if ENTRANCE_BEACON not in self.nodes:
            raise RuntimeError('Cannot connect the cable before the blue beacon is deployed')
        self.cable_connected = True
        self.flush()

    def disconnect_cable(self):
        self.cable_connected = False

    def route(self, source):
        path, links, seen = [], [], set()
        current = source
        while current != ENTRANCE_BEACON:
            if current not in self.nodes or current in seen:
                raise RuntimeError('No valid relay path to the entrance blue beacon')
            seen.add(current)
            path.append(current)
            node = self.nodes[current]
            parent = node['parent']
            if parent not in self.nodes:
                raise RuntimeError('Interior relay parent is missing')
            distance = math.dist(node['position'][:2], self.nodes[parent]['position'][:2])
            if distance > self.config['communication_range_metres']+1e-8:
                raise RuntimeError('Interior radio link is out of range')
            links.append({'from': current, 'to': parent, 'medium': 'wireless',
                          'distance_metres': round(distance, 5)})
            current = parent
        if ENTRANCE_BEACON not in self.nodes:
            raise RuntimeError('The entrance blue beacon is missing')
        path.extend((ENTRANCE_BEACON, OUTSIDE_GATEWAY))
        links.append({'from': ENTRANCE_BEACON, 'to': OUTSIDE_GATEWAY, 'medium': 'cable',
                      'length_metres': round(sum(math.dist(a, b) for a, b in
                                                zip(wire_vertices(self.config),
                                                    wire_vertices(self.config)[1:])), 5)})
        return path, links

    def send(self, record):
        # Derived delivery fields must not change the event's identity on retry.
        payload = deepcopy({k: v for k, v in record.items()
                            if k not in ('gateway_delivery', 'data_route')})
        key = record['id']
        existing = self.inbox.get(key) or self.pending.get(key)
        if existing:
            if existing['payload'] != payload:
                raise RuntimeError('Conflicting information for the same beacon event')
            return deepcopy(existing)
        path, links = self.route(record['beacon_name'])
        message = {'message_id': key, 'payload': payload, 'route': path,
                   'links': links, 'status': 'queued'}
        self.pending[key] = message
        self.flush()
        return deepcopy(self.inbox.get(key) or self.pending[key])

    def flush(self):
        if not self.cable_connected:
            return
        for key, message in list(self.pending.items()):
            path, links = self.route(message['payload']['beacon_name'])
            message.update(route=path, links=links, status='received')
            self.inbox[key] = message
            del self.pending[key]

    def require_received(self, records):
        expected = {record['id'] for record in records}
        if not self.cable_connected or set(self.inbox) != expected or self.pending:
            raise RuntimeError('Beacon information has not all reached the outside gateway through the cable')

    def snapshot(self):
        return deepcopy({'mode': 'logical_radio_relays_and_wired_uplink',
                         'outside_gateway': {'model': OUTSIDE_GATEWAY,
                                             'position': self.config['outside_gateway_position'],
                                             'direct_radio_to_interior': False},
                         'entrance_blue_beacon': ENTRANCE_BEACON,
                         'cable_connected': self.cable_connected,
                         'beacons': self.nodes,
                         'received_messages': list(self.inbox.values()),
                         'queued_messages': list(self.pending.values())})
