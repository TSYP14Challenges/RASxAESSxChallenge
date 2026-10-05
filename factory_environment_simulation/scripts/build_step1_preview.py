#!/usr/bin/env python3
"""Create a light Gazebo world for a timed, explicitly kinematic route demo."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from step1_timeline import Timeline, model_poses

PACKAGE = Path(__file__).resolve().parents[1]


def floating_link(source):
    link = deepcopy(source)
    for collision in list(link.findall('collision')):
        link.remove(collision)
    gravity = link.find('gravity')
    if gravity is None:
        gravity = ET.SubElement(link, 'gravity')
    gravity.text = 'false'
    return link


def preview_world(source, target, config):
    root = ET.parse(source).getroot()
    world = root.find('world')
    physics = world.find('physics')
    physics.find('max_step_size').text = str(config['preview_physics_step'])
    physics.find('real_time_factor').text = '1'
    # This first stage has no sensors; retain the renderer, commands and physics
    # systems needed to apply model poses, plus visible flame/smoke particles.
    for plugin in list(world.findall('plugin')):
        if plugin.get('name').split('::')[-1] in ('Sensors', 'Imu', 'Contact'):
            world.remove(plugin)
    robot = ET.parse(PACKAGE/'models/aisle_robot/model.sdf').getroot().find('model')
    links = {l.get('name'): l for l in robot.findall('link')}
    body = ET.Element('model', name=config['model_name'])
    ET.SubElement(body, 'static').text = 'false'
    ET.SubElement(body, 'allow_auto_disable').text = 'false'
    chassis = floating_link(links['chassis'])
    # Keep the small support ball visible as part of the same free body.
    support = deepcopy(links['support_ball'].find('visual'))
    support.set('name', 'support_ball_visual')
    c = list(map(float, links['chassis'].findtext('pose').split()))
    b = list(map(float, links['support_ball'].findtext('pose').split()))
    ET.SubElement(support, 'pose').text = f'{b[0]-c[0]} {b[1]-c[1]} {b[2]-c[2]} 0 0 0'
    chassis.append(support)
    body.append(chassis)
    body.append(deepcopy(next(p for p in robot.findall('plugin')
                              if p.get('name').endswith('::PosePublisher'))))
    models = [body]
    for side in ('left', 'right'):
        wheel = ET.Element('model', name=config['model_name']+'_'+side+'_wheel')
        ET.SubElement(wheel, 'static').text = 'false'
        ET.SubElement(wheel, 'allow_auto_disable').text = 'false'
        link = floating_link(links[side+'_wheel'])
        link.find('pose').text = '0 0 0 0 0 0'
        wheel.append(link)
        models.append(wheel)
    state = {'x': config['start'][0], 'y': config['start'][1], 'yaw': config['start'][2],
             'left_spin': 0.0, 'right_spin': 0.0}
    for model, pose in zip(models, model_poses(state, config['model_name'])):
        _, x, y, z, *_ = pose
        ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 {config["start"][2]}'
        world.append(model)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)


def build(config):
    preview_world(PACKAGE/'worlds/warehouse_lite.sdf',
                  PACKAGE/'worlds/warehouse_step1_preview.sdf', config)
