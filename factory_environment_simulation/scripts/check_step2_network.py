#!/usr/bin/env python3
"""Verify that beacon information needs the entrance cable to reach outside."""
from copy import deepcopy
import unittest

from step2_events import load_config
from step2_network import BeaconNetwork, ENTRANCE_BEACON, OUTSIDE_GATEWAY


class NetworkChecks(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.network = BeaconNetwork(self.config)
        self.blue = self.record('gateway', 'gateway', ENTRANCE_BEACON,
                                self.config['entrance_beacon_position'], None)

    def record(self, event, kind, name, position, parent):
        return {'id': event, 'kind': kind, 'beacon_name': name,
                'position': list(position), 'parent_beacon': parent,
                'robot_pose': [0., position[1], 0.], 'pause_seconds': 1.}

    def test_outside_gateway_alone_cannot_provide_inside_radio(self):
        record = self.record('fire_01', 'fire', 'fire_tile', [-.45, -11., .016], OUTSIDE_GATEWAY)
        with self.assertRaisesRegex(RuntimeError, 'No deployed interior relay'):
            self.network.register(record)
        self.assertEqual(self.network.snapshot()['received_messages'], [])
        self.assertFalse(self.network.snapshot()['outside_gateway']['direct_radio_to_interior'])

    def test_cable_cannot_connect_before_the_robot_drops_blue_beacon(self):
        with self.assertRaisesRegex(RuntimeError, 'before the blue beacon'):
            self.network.connect_cable()

    def test_blue_and_nearby_fire_information_stays_queued_without_cable(self):
        self.network.register(self.blue)
        self.assertEqual(self.network.send(self.blue)['status'], 'queued')
        fire = self.record('fire_01', 'fire', 'fire_tile', [-.45, -9., .016], ENTRANCE_BEACON)
        self.network.register(fire)
        self.assertEqual(self.network.send(fire)['status'], 'queued')
        self.assertEqual(self.network.inbox, {})
        self.network.connect_cable()
        self.network.require_received([self.blue, fire])
        self.assertEqual(len(self.network.inbox), 2)
        self.assertEqual(self.network.inbox['fire_01']['route'],
                         ['fire_tile', ENTRANCE_BEACON, OUTSIDE_GATEWAY])
        self.assertEqual(self.network.inbox['fire_01']['links'][-1]['medium'], 'cable')

    def test_distant_beacon_relays_each_hop_then_uses_cable(self):
        self.network.register(self.blue)
        self.network.connect_cable()
        parent = ENTRANCE_BEACON
        for event, kind, y in [('range_01', 'range', -4.), ('range_02', 'range', 3.),
                               ('fire_01', 'fire', 10.)]:
            record = self.record(event, kind, event+'_tile', [-.45, y, .016], parent)
            self.network.register(record)
            delivered = self.network.send(record)
            parent = record['beacon_name']
        self.assertEqual(delivered['route'],
                         ['fire_01_tile', 'range_02_tile', 'range_01_tile',
                          ENTRANCE_BEACON, OUTSIDE_GATEWAY])
        self.assertEqual([link['medium'] for link in delivered['links']],
                         ['wireless', 'wireless', 'wireless', 'cable'])
        self.assertTrue(all(link['distance_metres'] <= 8 for link in delivered['links'][:-1]))
        self.assertEqual(delivered['payload']['position'], [-.45, 10., .016])

    def test_link_beyond_radio_range_cannot_register(self):
        self.network.register(self.blue)
        x, y, z = self.blue['position']
        far = self.record('far_fire', 'fire', 'far_tile',
                          [x, y+self.config['communication_range_metres']+1., z], ENTRANCE_BEACON)
        with self.assertRaisesRegex(RuntimeError, 'exceeds the beacon range'):
            self.network.register(far)
        self.assertNotIn('far_tile', self.network.nodes)

    def test_cable_loss_blocks_new_information_and_reconnection_delivers_it(self):
        self.network.register(self.blue)
        self.network.connect_cable()
        self.network.send(self.blue)
        self.network.disconnect_cable()
        victim = self.record('victim_alive', 'victim', 'victim_tile', [-.45, -7., .016], ENTRANCE_BEACON)
        victim['victim_status'] = 'alive'
        self.network.register(victim)
        self.assertEqual(self.network.send(victim)['status'], 'queued')
        self.assertNotIn('victim_alive', self.network.inbox)
        with self.assertRaisesRegex(RuntimeError, 'through the cable'):
            self.network.require_received([self.blue, victim])
        self.network.connect_cable()
        self.network.require_received([self.blue, victim])
        self.assertEqual(self.network.inbox['victim_alive']['payload']['victim_status'], 'alive')

    def test_same_information_is_received_once_and_conflicting_retry_is_rejected(self):
        self.network.register(self.blue)
        self.network.connect_cable()
        first = self.network.send(self.blue)
        self.assertEqual(self.network.send(self.blue), first)
        self.assertEqual(len(self.network.inbox), 1)
        changed = deepcopy(self.blue)
        changed['position'][0] += .2
        with self.assertRaisesRegex(RuntimeError, 'Conflicting information'):
            self.network.send(changed)


if __name__ == '__main__':
    unittest.main()
