#!/usr/bin/env python3
"""Add one gray copy of the working robot; keep the factory and entrance assembly."""
from copy import deepcopy
import hashlib
import json
import xml.etree.ElementTree as ET

from build_step2 import floating_model, material
from step1_timeline import model_poses
from step2_events import PACKAGE
from step3_scenario import load_config


def ball_name(fire_id):
    return 'step3_'+fire_id+'_ball'


def ball_sdf(name, position, config):
    model, link = floating_model(name, position)
    model.find('plugin/update_frequency').text = '30'
    visual = ET.SubElement(link, 'visual', name='extinguisher_ball')
    sphere = ET.SubElement(ET.SubElement(visual, 'geometry'), 'sphere')
    ET.SubElement(sphere, 'radius').text = str(config['ball_radius_metres'])
    material(visual, [.96, .98, 1., 1.], emission=.15)
    root = ET.Element('sdf', version='1.9')
    root.append(model)
    return ET.tostring(root, encoding='unicode')


def extinguished_sdf(source):
    """Keep the same charred objects and collisions, without flame, smoke or glow."""
    model = deepcopy(source)
    for link in model.findall('link'):
        for tag in ('particle_emitter', 'light'):
            for element in list(link.findall(tag)):
                link.remove(element)
    pose = list(map(float, model.findtext('pose').split()))[:3]
    publisher, _ = floating_model(model.get('name'), pose, fixed=True)
    model.append(publisher.find('plugin'))
    root = ET.Element('sdf', version='1.9')
    root.append(model)
    return ET.tostring(root, encoding='unicode')


def main():
    config = load_config()
    directory = PACKAGE/'step3'
    directory.mkdir(exist_ok=True)
    source = PACKAGE/'worlds/warehouse_step2.sdf'
    root = ET.parse(source).getroot()
    world = root.find('world')
    models = {m.get('name'): m for m in world.findall('model')}
    initial = {'x': config['start'][0], 'y': config['start'][1], 'yaw': config['start'][2],
               'left_spin': 0., 'right_spin': 0.}
    for source_name, pose in zip(('aisle_robot', 'aisle_robot_left_wheel', 'aisle_robot_right_wheel'),
                                 model_poses(initial, config['model_name'])):
        model = deepcopy(models[source_name])
        model.set('name', pose[0])
        model.find('pose').text = f'{pose[1]} {pose[2]} {pose[3]} 0 0 {config["start"][2]}'
        cube = model.find('link/visual[@name="black_cube"]')
        if cube is not None:
            cube.set('name', 'gray_cube')
            for tag in ('ambient', 'diffuse'):
                cube.find('material/'+tag).text = ' '.join(map(str, config['gray_color']))
        world.append(model)
    fire_ids = sorted(name for name in models if name.startswith('fire_'))
    for name in fire_ids:
        (directory/('extinguished_'+name+'.sdf')).write_text(extinguished_sdf(models[name])+'\n')
    (directory/'example_ball.sdf').write_text(ball_sdf(ball_name(fire_ids[0]), [1.2, -14., .7], config)+'\n')
    ET.indent(root)
    target = PACKAGE/'worlds/warehouse_step3.sdf'
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    manifest = {'source_step2_world_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'config_sha256': hashlib.sha256((directory/'config.json').read_bytes()).hexdigest(),
                'world_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                'gray_robot': config['model_name'], 'gray_start': config['start'],
                'fire_ids': fire_ids, 'balls': [ball_name(name) for name in fire_ids],
                'start_order': ['writer_returned_outside', 'gray_robot_enters'],
                'beacons_preserved': True}
    (directory/'scene.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Built step 3: gray robot waits outside; six fire balls; same victim tiles change to green/red.')


if __name__ == '__main__':
    main()
