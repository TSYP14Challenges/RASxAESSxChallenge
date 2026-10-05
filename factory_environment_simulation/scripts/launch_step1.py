#!/usr/bin/env python3
"""Start Gazebo and its small controller together; clean up only this demo's processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from step1_settings import load_config, motion_arguments, validate_motion

PACKAGE = Path(__file__).resolve().parents[1]


def runtime_world(source, target, time_factor):
    """Change the requested clock rate while preserving the physics step and scene."""
    root = ET.parse(source).getroot()
    physics = root.find('world/physics')
    factor = physics.find('real_time_factor')
    if factor is None:
        factor = ET.SubElement(physics, 'real_time_factor')
    factor.text = str(time_factor)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    return target


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
    except ProcessLookupError:
        pass


def main():
    config = load_config(PACKAGE)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lite', action='store_true', help='Fewer particles and disabled shadows')
    parser.add_argument('--headless', action='store_true', help='Physics/server only')
    parser.add_argument('--physics', action='store_true', help='Use the previous wheel-physics controller')
    parser.add_argument('--seconds', type=float, default=config['preview_duration_seconds'],
                        help='Target real seconds for the complete animated tour (default: 90)')
    parser.add_argument('--demo', action='store_true',
                        help='Fast animated tour; with --physics, request the previous 6x clock')
    parser.add_argument('--time-factor', type=float, default=None,
                        help='Requested simulation / real-time ratio, 0.1 to 12')
    motion_arguments(parser, config)
    args = parser.parse_args()
    validate_motion(args)
    factor = args.time_factor if args.time_factor is not None else \
        config['demo_time_factor'] if args.demo and args.physics else 1.0
    if not 0.1 <= factor <= 12:
        raise ValueError('--time-factor must be between 0.1 and 12')
    if args.physics:
        from drive_step1 import transport_imports
    else:
        from animate_step1 import transport_imports
        from step1_timeline import Timeline
        import json
        if not 10 <= args.seconds <= 600:
            raise ValueError('--seconds must be between 10 and 600')
        Timeline(json.loads((PACKAGE/'step1/route.json').read_text())['waypoints'],
                 config, args.seconds, args.speed)
    transport_imports()
    subprocess.run([sys.executable, str(PACKAGE/'scripts/check_step1.py'), '--structural-only'], check=True)
    if not args.physics:
        subprocess.run([sys.executable, str(PACKAGE/'scripts/check_step1_preview.py'),
                        '--structural-only'], check=True)
    world = PACKAGE/'worlds'/('warehouse_step1_lite.sdf' if args.lite or args.demo else 'warehouse_step1.sdf') \
        if args.physics else PACKAGE/'worlds/warehouse_step1_preview.sdf'
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    if factor != 1.0:
        world = runtime_world(world, logs/'warehouse_step1_runtime.sdf', factor)
    command = ['gz', 'sim', '-r', '-v', '3']
    if args.headless:
        command.append('-s')
    command.append(str(world))
    gazebo, controller = None, None
    try:
        with (logs/'gazebo-step1.log').open('w') as output:
            print('Opening your factory environment and the step 1 robot...', flush=True)
            if args.physics:
                print(f'Robot speed: {args.speed if args.speed is not None else config["linear_speed"]:g} m/s; '
                      f'startup wait: {args.start_delay:g} sim seconds; requested time factor: {factor:g}x.', flush=True)
            else:
                print(f'Mode: fast route animation; target: {args.seconds:g} real seconds; '
                      f'startup wait: {args.start_delay:g} real seconds.', flush=True)
            if factor > 1:
                print('Accelerated demo: the actual clock rate depends on the computer. '
                      'The final report includes real elapsed time.', flush=True)
            gazebo = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            controller_command = [sys.executable, '-u', str(PACKAGE/'scripts'/
                                  ('drive_step1.py' if args.physics else 'animate_step1.py')),
                                  '--start-delay', str(args.start_delay)]
            if not args.physics:
                controller_command += ['--seconds', str(args.seconds)]
            if args.speed is not None:
                controller_command += ['--speed', str(args.speed)]
            controller = subprocess.Popen(controller_command, start_new_session=True)
            while controller.poll() is None and gazebo.poll() is None:
                time.sleep(0.15)
            if gazebo.poll() is not None and controller.poll() is None:
                print('Gazebo closed. Stopping the controller.', flush=True)
                stop_process(controller)
                if gazebo.returncode:
                    print('Inspect '+str(logs/'gazebo-step1.log'), file=sys.stderr)
                return gazebo.returncode
            if controller.returncode:
                print('The tour stopped. Details: '+str(logs/'step1_result.json'), file=sys.stderr)
                print('Gazebo log: '+str(logs/'gazebo-step1.log'), file=sys.stderr)
                if not args.physics:
                    print('Pose worker log: '+str(logs/'step1_pose_worker.log'), file=sys.stderr)
                return controller.returncode
            if args.headless:
                return 0
            print('Robot stopped outside. The Gazebo window remains open; close it or press Ctrl+C.', flush=True)
            gazebo.wait()
            return gazebo.returncode
    except KeyboardInterrupt:
        print('\nClosing this demo.', flush=True)
        return 130
    finally:
        stop_process(controller)
        stop_process(gazebo)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('STEP 1 LAUNCH ERROR:', error, file=sys.stderr)
        sys.exit(1)
