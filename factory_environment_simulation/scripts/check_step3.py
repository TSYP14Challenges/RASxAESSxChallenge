#!/usr/bin/env python3
"""Check the complete two-robot scenario using geometry and transport fixtures."""
import argparse
from collections import Counter
from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from animate_step3 import Step3Connection, extra_message_types, run_executor
from build_step3 import ball_name
from check_step2 import CreationFixture, FakeFactory, MissionChecks, Timer, fixture_connection, normalized
from check_step1_preview import FakePoseV
from step1_pose_service import PoseServiceProcess
from step2_events import PACKAGE, load_config as writer_config
from step3_scenario import AccessibleWriterEvents, load_config, plan_executor, validate_handoff


def structural_check():
    config = load_config()
    manifest = json.loads((PACKAGE/'step3/scene.json').read_text())
    for key, name in (('source_step2_world_sha256', 'worlds/warehouse_step2.sdf'),
                      ('world_sha256', 'worlds/warehouse_step3.sdf'), ('config_sha256', 'step3/config.json')):
        if manifest[key] != hashlib.sha256((PACKAGE/name).read_bytes()).hexdigest():
            raise RuntimeError('Step 3 scene changed. Run bash run.sh plan first.')
    source = ET.parse(PACKAGE/'worlds/warehouse_step2.sdf').find('world')
    target = ET.parse(PACKAGE/'worlds/warehouse_step3.sdf').find('world')
    models = {m.get('name'): m for m in target.findall('model')}
    assert len(models) == len(target.findall('model'))
    added = {config['model_name'], config['model_name']+'_left_wheel', config['model_name']+'_right_wheel'}
    assert {m.get('name') for m in target.findall('model')}-{m.get('name') for m in source.findall('model')} == added
    assert [normalized(m) for m in source.findall('model')] == \
        [normalized(m) for m in target.findall('model') if m.get('name') not in added], \
        'Step 3 changed the working factory, writer or entrance cable'
    gray = models[config['model_name']]
    assert list(map(float, gray.findtext('pose').split()))[:2] == config['start'][:2]
    assert config['start'][1] < -12. and math.dist(config['start'][:2], writer_config()['start'][:2]) > 1.
    for tag in ('ambient', 'diffuse'):
        assert list(map(float, gray.findtext('link/visual[@name="gray_cube"]/material/'+tag).split())) == config['gray_color']
    for name in added:
        assert not models[name].findall('.//collision')
        assert all(link.findtext('gravity') == 'false' for link in models[name].findall('link'))
    assert 'step2_entrance_blue_beacon' in models
    assert {v.get('name') for v in models['step2_entrance_blue_beacon'].findall('link/visual')} == \
        {'tile', 'cable_0', 'cable_1'}
    assert len(manifest['fire_ids']) == 6 and not set(manifest['balls']) & set(models)
    for name in manifest['fire_ids']:
        off = ET.parse(PACKAGE/'step3'/('extinguished_'+name+'.sdf')).find('model')
        before = models[name]
        assert off.get('name') == name and off.findtext('pose') == before.findtext('pose')
        assert not off.findall('.//particle_emitter') and not off.findall('.//light')
        assert [normalized(c) for c in off.findall('.//collision')] == \
            [normalized(c) for c in before.findall('.//collision')]
        assert [normalized(v) for v in off.findall('.//visual')] == \
            [normalized(v) for v in before.findall('.//visual')]
        assert off.findtext('plugin/topic') == '/model/'+name+'/pose'
    ball = ET.parse(PACKAGE/'step3/example_ball.sdf').find('model')
    assert float(ball.findtext('link/visual/geometry/sphere/radius')) == config['ball_radius_metres']
    assert float(ball.findtext('pose').split()[2]) > config['ball_radius_metres']
    print('PASS: original factory/writer/blue cable preserved; gray robot parked outside; '
          'six fire definitions preserve charred objects without flame, smoke or glow.')
    return config


class EntityStub:
    MODEL = 2
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class VisualStub:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
        self.material = Obj(**{key: Obj(r=0., g=0., b=0., a=1.)
                              for key in ('ambient', 'diffuse', 'emissive')})


class WorldFixture(CreationFixture):
    """Apply real generated SDF and in-place visual commands to an offline scene."""
    def __init__(self, connection, beacons, timer):
        super().__init__(connection)
        self.timer, self.ids, self.visual_ids, self.next_id = timer, {}, {}, 100
        self.commands, self.ball_positions = [], []
        self.fail_colors = False
        for model in ET.parse(PACKAGE/'worlds/warehouse_step3.sdf').findall('world/model'):
            root = ET.Element('sdf', version='1.9')
            root.append(model)
            message = FakeFactory()
            message.name, message.sdf = model.get('name'), ET.tostring(root, encoding='unicode')
            self.models[message.name] = message
        self.models.update(deepcopy(beacons))
        for name in self.models:
            self.allocate(name)

    def allocate(self, name):
        self.next_id += 1
        self.ids[name] = self.next_id
        for link in ET.fromstring(self.models[name].sdf).findall('model/link'):
            self.next_id += 1
            parent = self.next_id
            for visual in link.findall('visual'):
                self.next_id += 1
                self.visual_ids[(name, link.get('name')+'::'+visual.get('name'))] = (self.next_id, parent)

    def scene_models(self, service, names, timeout=2000):
        assert service == '/world/factory/scene/info'
        result = {}
        for name in names:
            if name not in self.models:
                continue
            visuals = {}
            for link in ET.fromstring(self.models[name].sdf).findall('model/link'):
                for visual in link.findall('visual'):
                    key = link.get('name')+'::'+visual.get('name')
                    vid, parent = self.visual_ids[(name,key)]
                    text = visual.findtext('material/diffuse', '0 0 0 1')
                    visuals[key] = {'id':vid, 'parent_id':parent, 'diffuse':list(map(float,text.split()))}
            result[name] = {'id':self.ids[name], 'visuals':visuals}
        return result

    def request(self, service, message, request_type, response_type, timeout):
        self.commands.append((self.timer.wall,service,deepcopy(message)))
        if service == '/world/factory/create':
            result = super().request(service,message,request_type,response_type,timeout)
            if message.name not in self.ids:
                self.allocate(message.name)
            return result
        if service == '/world/factory/remove':
            assert request_type is EntityStub and message.type == EntityStub.MODEL
            assert self.ids[message.name] == message.id
            self.models.pop(message.name)
            self.ids.pop(message.name)
            return True,Obj(data=True)
        if service == '/world/factory/visual_config':
            assert request_type is VisualStub
            if self.fail_colors:
                return True,Obj(data=False)
            name,key = next(key for key,value in self.visual_ids.items() if value[0] == message.id)
            root = ET.fromstring(self.models[name].sdf)
            link_name, visual_name = key.split('::')
            visual = root.find(f'model/link[@name="{link_name}"]/visual[@name="{visual_name}"]')
            for tag in ('ambient','diffuse','emissive'):
                element = visual.find('material/'+tag)
                if element is None:
                    element = ET.SubElement(visual.find('material'),tag)
                color = getattr(message.material,tag)
                element.text = f'{color.r} {color.g} {color.b} {color.a}'
            self.models[name].sdf = ET.tostring(root,encoding='unicode')
            return True,Obj(data=True)
        if service == '/world/factory/set_pose/blocking':
            self.ball_positions.append((self.timer.wall,message.name,
                                        [message.position.x,message.position.y,message.position.z]))
            root = ET.fromstring(self.models[message.name].sdf)
            root.find('model/pose').text = f'{message.position.x} {message.position.y} {message.position.z} 0 0 0'
            self.models[message.name].sdf = ET.tostring(root,encoding='unicode')
            self.feedback(self.models[message.name])
            return True,Obj(data=True)
        raise AssertionError('Unexpected fixture service: '+service)


class ScenarioChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch('check_step2.MissionEvents',AccessibleWriterEvents), redirect_stdout(io.StringIO()):
            cfg,route,mission,connection,report,output = MissionChecks().mission(
                radio_range=writer_config()['communication_range_metres'])
        cls.writer,cls.records,cls.beacons = report,mission.records,connection.requester.models
        cls.config = load_config()
        cls.plan,cls.grid = plan_executor(cls.config,cls.records,cls.writer)
        (PACKAGE/'step3/offline_writer.json').write_text(json.dumps({**report,'native_gazebo_runtime':False},indent=2)+'\n')
        (PACKAGE/'step3/offline_plan.json').write_text(json.dumps(cls.plan,indent=2)+'\n')
        cls.grid.save(PACKAGE/'step3/offline_map.json')

    def connection(self, timer, delay=0., paused=False, lost=False, never=False, writer_inside=False):
        test = self
        class Connection(Step3Connection):
            def __init__(self):
                fixture_connection(self,ScenarioChecks.config)
                self.model_names |= {a['id'] for a in ScenarioChecks.plan['actions'] if a['kind']=='fire'}
                self.model_names |= {ball_name(a['id']) for a in ScenarioChecks.plan['actions'] if a['kind']=='fire'}
                self.model_names.add(writer_config()['model_name'])
                self.tracked_models |= self.model_names
                self.Pose = lambda: FakePoseV().pose.add()
                self.Entity,self.Visual = EntityStub,VisualStub
                self.request_failures = self.request_retries = 0
                self.last_failure = None
                self.service = '/world/factory/set_pose_vector/blocking'
                self.single_service = '/world/factory/set_pose/blocking'
                self.scene_service = '/world/factory/scene/info'
                self.visual_service = '/world/factory/visual_config'
                self.remove_service = '/world/factory/remove'
                self.requester = WorldFixture(self,ScenarioChecks.beacons,timer)
                self.victim_visuals = {}
                self.pose = {'x':self.config['start'][0],'y':self.config['start'][1],
                             'yaw':self.config['start'][2],'left_spin':0.,'right_spin':0.}
                self.pending,self.due = None,0.
                self.sent,self.action_times = [],[]
                self.dropped_motion = False
                x,y,_ = writer_config()['start']
                self.model_samples[writer_config()['model_name']] = (x,-11. if writer_inside else y,.005,0.)
            def state(self):
                if self.pending is not None and timer.wall >= self.due:
                    self.pose,self.pending = self.pending,None
                sim = min(timer.wall,2.) if paused and timer.wall < 4. else timer.wall-(2. if paused else 0.)
                return {**self.pose,'z':.005,'sim_time':sim,'pose_time':sim,
                        'pose_fresh':True,'time_reset':False}
            def send(self, pose):
                self.command_pose_time = self.state()['sim_time']+2*self.config['preview_physics_step']
                if self.sent:
                    previous = self.sent[-1][1]
                    test.assertLessEqual(math.dist((previous['x'],previous['y']),(pose['x'],pose['y'])),.250001)
                self.sent.append((timer.wall,deepcopy(pose)))
                moving = math.dist((self.pose['x'],self.pose['y']),(pose['x'],pose['y'])) > .002
                if moving and (never or (lost and not self.dropped_motion)):
                    self.dropped_motion = True
                    return
                self.pending,self.due = deepcopy(pose),timer.wall+delay
            def perform_action(self,action,pose,clock=None):
                self.action_times.append((timer.wall,action['id'],deepcopy(pose)))
                return super().perform_action(action,pose,timer if clock is None else clock)
        return Connection()

    def mission(self, **kwargs):
        timer = Timer()
        connection = self.connection(timer,**kwargs)
        report = {'completed':False,'native_gazebo_runtime':False}
        with patch('animate_step2.time',timer), redirect_stdout(io.StringIO()):
            run_executor(connection,self.config,self.plan,self.grid,report,clock=timer)
        connection.verify_beacons(self.records)
        return timer,connection,report

    def test_full_two_robot_scenario_extinguishes_six_fires_and_recolors_same_victim_tiles(self):
        timer,connection,report = self.mission()
        self.assertTrue(self.writer['completed'] and report['completed'])
        self.assertEqual(len(self.writer['visited_sections']),15)
        self.assertEqual(len(report['actions']),8)
        self.assertEqual(Counter(a['kind'] for a in report['actions']),{'fire':6,'victim':2})
        self.assertEqual([a['kind'] for a in report['actions']],['victim']*2+['fire']*6)
        by_id = {a['id']:a for a in report['actions']}
        self.assertEqual(by_id['victim_alive']['beacon_color'],'green')
        self.assertEqual(by_id['victim_dead']['beacon_color'],'red')
        self.assertLess(report['finish_error_metres'],.00001)
        self.assertLess(report['elapsed_wall_seconds'],150.)
        for action in report['actions']:
            self.assertGreaterEqual(action['pause_seconds']+1e-8,1.)
            self.assertLessEqual(math.dist(action['robot_pose'][:2],action['beacon_position'][:2]),.002)
        for name in ('victim_alive','victim_dead'):
            model = connection.requester.scene_models('/world/factory/scene/info',['step2_'+name+'_beacon'])
            color = model['step2_'+name+'_beacon']['visuals']['body::tile']['diffuse']
            self.assertEqual(color,self.config['alive_beacon_color' if name=='victim_alive' else 'dead_beacon_color'])
        self.assertTrue(set(self.beacons) <= set(connection.requester.models))
        self.assertFalse([cmd for _,service,cmd in connection.requester.commands
                          if service.endswith('/remove') and cmd.name in self.beacons])
        self.assertFalse(set(self.beacons) & {m.name for m in connection.requester.calls})
        for name in connection.requester.models:
            if name.startswith('fire_'):
                model = ET.fromstring(connection.requester.models[name].sdf).find('model')
                self.assertFalse(model.findall('.//particle_emitter') or model.findall('.//light'))
                self.assertTrue(model.findall('.//collision'))
        balls = [name for name in connection.requester.models if name.startswith('step3_fire_')]
        self.assertEqual(len(balls),6)
        for name in balls:
            flight = [position for _,ball,position in connection.requester.ball_positions if ball == name]
            self.assertEqual(len(flight),30)
            self.assertGreater(max(p[2] for p in flight),1.)
            fire = next(a for a in self.plan['actions'] if ball_name(a['id']) == name)
            self.assertLess(math.dist(flight[-1][:2],fire['event_centre']),1e-8)
            self.assertAlmostEqual(flight[-1][2],self.config['ball_radius_metres']+.01)
            removal = next(t for t,service,message in connection.requester.commands
                           if service.endswith('/remove') and message.name == fire['id'])
            impact = max(t for t,ball,_ in connection.requester.ball_positions if ball == name)
            self.assertGreaterEqual(removal,impact)
        (PACKAGE/'step3/offline_mission.json').write_text(json.dumps({
            'completed':True,'native_gazebo_runtime':False,'writer':self.writer,'executor':report,
            'note':'Real writer and gray mission loops, SDF creation, fire replacement and existing-visual color requests with offline transport fixtures.'},indent=2)+'\n')

    def test_route_avoids_blue_yellow_tiles_and_uses_exact_actual_beacon_centres(self):
        self.assertEqual({a['kind'] for a in self.plan['actions']},{'fire','victim'})
        self.assertEqual([a['kind'] for a in self.plan['actions']],['victim']*2+['fire']*6)
        self.assertEqual({a['id'] for a in self.plan['actions'][:2]},
                         {'victim_alive','victim_dead'})
        self.assertTrue(all(self.grid.line_safe((a['x'],a['y']),(b['x'],b['y']))
                            for a,b in zip(self.plan['waypoints'],self.plan['waypoints'][1:])))
        for action in self.plan['actions']:
            waypoint = next(w for w in self.plan['waypoints'] if w['kind']=='action' and w['name']==action['id'])
            self.assertEqual([waypoint['x'],waypoint['y']],action['position'][:2])
        self.assertEqual(set(self.plan['ignored_beacons']),
                         {b['beacon_name'] for b in self.records if b['kind'] in ('gateway','range')})

    def test_fewer_yellow_tiles_only_fill_gaps_between_all_existing_beacon_colors(self):
        limit = writer_config()['communication_range_metres']
        yellow = [b for b in self.records if b['kind']=='range']
        self.assertGreater(len(yellow),0)
        self.assertLess(len(yellow),11, 'The previous default writer run used 11 yellow tiles')
        placed = []
        for beacon in self.records:
            if beacon['kind']=='range':
                self.assertGreater(min(math.dist(beacon['robot_pose'][:2],b['position'][:2])
                                       for b in placed),limit)
            if placed:
                self.assertLessEqual(beacon['parent_distance_metres'],limit)
            placed.append(beacon)
        self.assertEqual({b['kind']:sum(r['kind']==b['kind'] for r in self.records)
                          for b in self.records if b['kind']!='range'},
                         {'gateway':1,'fire':6,'victim':2})
        self.assertEqual({p['message_id'] for p in self.writer['communication']['received_messages']},
                         {b['id'] for b in self.records})

    def test_gray_robot_does_not_enter_before_writer_returns_or_with_missing_target_data(self):
        with self.assertRaisesRegex(RuntimeError,'writer must return'):
            validate_handoff({**self.writer,'completed':False},self.records)
        with self.assertRaisesRegex(RuntimeError,'all six fire'):
            validate_handoff(self.writer,self.records[:-1])
        timer = Timer()
        connection = self.connection(timer,writer_inside=True)
        with patch('animate_step2.time',timer),redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'gray robot remains parked'):
                run_executor(connection,self.config,self.plan,self.grid,{},clock=timer)
        self.assertEqual(connection.sent,[])

    def test_delayed_feedback_and_one_recovery_complete_without_duplicate_balls(self):
        _,connection,report = self.mission(delay=.06,lost=True)
        self.assertTrue(report['completed'])
        self.assertEqual(connection.motion_recoveries,1)
        self.assertEqual(len({a['id'] for a in report['actions']}),8)
        self.assertEqual(len([m for m in connection.requester.calls if m.name.startswith('step3_fire_')]),6)
        self.assertLess(report['elapsed_wall_seconds'],180.)

    def test_paused_clock_holds_gray_motion(self):
        _,connection,report = self.mission(paused=True)
        self.assertTrue(report['completed'])
        self.assertFalse([t for t,_ in connection.sent if 2.6 < t < 3.9])

    def test_unapplied_gray_movement_stops_without_marking_or_extinguishing(self):
        timer = Timer(); connection = self.connection(timer,never=True)
        report = {'completed':False}
        with patch('animate_step2.time',timer),redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'Gray robot did not reach target'):
                run_executor(connection,self.config,self.plan,self.grid,report,clock=timer)
        self.assertEqual(report['actions'],[])
        self.assertFalse(report['completed'])

    def test_rejected_color_request_does_not_report_a_completed_victim_action(self):
        timer = Timer();connection = self.connection(timer)
        connection.requester.fail_colors = True
        victim = next(a for a in self.plan['actions'] if a['kind']=='victim')
        with patch('animate_step2.time',timer),redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'rejected the victim beacon color'):
                connection.color_victim(victim)
        self.assertEqual(connection.victim_visuals,{})

    def test_no_action_can_run_away_from_the_beacon_or_on_blue_yellow(self):
        timer = Timer();connection = self.connection(timer)
        with self.assertRaisesRegex(RuntimeError,'stand above'):
            connection.perform_action(self.plan['actions'][0],{'x':1.2,'y':-14.,'yaw':0.},timer)
        with self.assertRaisesRegex(RuntimeError,'not action targets'):
            connection.perform_action({'id':'range_01','kind':'range','position':[1.2,-14.,.016]},
                                      {'x':1.2,'y':-14.,'yaw':0.},timer)


class VisualProbe:
    DESCRIPTOR = Obj(full_name='gz.msgs.Visual')
    def __init__(self,**payload): self.payload = payload
    def SerializeToString(self): return json.dumps(self.payload).encode()

class EntityProbe(VisualProbe):
    DESCRIPTOR = Obj(full_name='gz.msgs.Entity')


WORKER_TRANSPORT = '''from types import SimpleNamespace as O
models = {"step2_victim_alive_beacon": [100,101,102,[1.,1.,1.,1.]], "fire_01":[200,201,202,[.1,.1,.1,1.]]}
class Node:
    def request(self,service,message,request_type,response_type,timeout):
        if service.endswith('/scene/info'):
            result=[]
            for name,(mid,lid,vid,c) in models.items():
                visual=O(name='tile',id=vid,material=O(diffuse=O(r=c[0],g=c[1],b=c[2],a=c[3])))
                result.append(O(name=name,id=mid,link=[O(name='body',id=lid,visual=[visual])]))
            return True,O(model=result)
        if service.endswith('/visual_config'):
            for value in models.values():
                if value[2]==message.payload['id']:value[3]=message.payload['color']
            return True,O(data=True)
        if service.endswith('/remove'):
            models.pop(message.payload['name'])
            return True,O(data=True)
        raise RuntimeError('Unexpected service in scene worker fixture')
'''


class SceneWorkerChecks(unittest.TestCase):
    def test_real_worker_queries_scene_ids_and_decodes_new_visual_entity_types(self):
        with tempfile.TemporaryDirectory(prefix='step3_scene_worker_') as directory:
            root = Path(directory);msgs=root/'gz/msgs10';msgs.mkdir(parents=True)
            for path in (msgs/'__init__.py',msgs.parent/'__init__.py'):path.write_text('')
            (msgs.parent/'transport13.py').write_text(WORKER_TRANSPORT)
            (msgs/'fixture.py').write_text('import json\nclass Message:\n    def ParseFromString(self,data): self.payload=json.loads(data)\n')
            for module,name in (('pose_pb2','Pose'),('pose_v_pb2','Pose_V'),('boolean_pb2','Boolean'),
                                ('visual_pb2','Visual'),('entity_pb2','Entity'),('empty_pb2','Empty'),('scene_pb2','Scene')):
                (msgs/(module+'.py')).write_text('from .fixture import Message\nclass '+name+'(Message):pass\n')
            with patch.dict(os.environ,{'PYTHONPATH':str(root),'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION':'python'}):
                worker=PoseServiceProcess(root/'service.log')
                try:
                    first=worker.scene_models('/world/factory/scene/info',['step2_victim_alive_beacon'])
                    self.assertEqual(first['step2_victim_alive_beacon']['visuals']['body::tile']['id'],102)
                    result,reply=worker.request('/world/factory/visual_config',VisualProbe(id=102,color=[.05,.85,.15,1.]),VisualProbe,None,2000)
                    self.assertTrue(result and reply.data)
                    changed=worker.scene_models('/world/factory/scene/info',['step2_victim_alive_beacon'])
                    self.assertEqual(changed['step2_victim_alive_beacon']['visuals']['body::tile']['id'],102)
                    self.assertEqual(changed['step2_victim_alive_beacon']['visuals']['body::tile']['diffuse'],[.05,.85,.15,1.])
                    result,reply=worker.request('/world/factory/remove',EntityProbe(name='fire_01',type=2),EntityProbe,None,2000)
                    self.assertTrue(result and reply.data)
                    self.assertEqual(worker.scene_models('/world/factory/scene/info',['fire_01']),{})
                finally:
                    worker.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--structural-only',action='store_true')
    parser.add_argument('--native',action='store_true')
    args=parser.parse_args()
    structural_check()
    if args.structural_only:return
    if args.native:
        extra_message_types()
        print('PASS: native Gazebo Entity, Visual/material and Scene IDs decode correctly (Python parser).')
        return
    suite=unittest.TestSuite()
    for cls in (ScenarioChecks,SceneWorkerChecks):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(cls))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    record={'passed':result.wasSuccessful(),'tests':result.testsRun,'native_gazebo_runtime_tested_here':False,
            'writer_before_executor':True,'all_six_fires_extinguished':True,
            'left_victim_green_right_victim_red':True,'victim_visual_ids_preserved':True,
            'blue_yellow_tiles_avoided_and_preserved':True,'gray_stands_above_actual_beacon_centres':True,
            'full_native_gui_mission_requires_target_pc':True}
    (PACKAGE/'step3/validation.json').write_text(json.dumps(record,indent=2)+'\n')
    if not result.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':
    main()
