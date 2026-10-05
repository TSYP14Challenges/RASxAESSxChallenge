#!/usr/bin/env python3
"""One launch: black writer, automatic handoff, gray executor, final scene stays open."""
import argparse
import json
import subprocess
import sys
import time

from animate_step3 import extra_message_types
from launch_step1 import stop_process
from step2_events import PACKAGE, load_config as writer_config
from step3_scenario import load_config


def main():
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--writer-seconds', type=float, default=writer_config()['motion_seconds'])
    parser.add_argument('--seconds', type=float, default=config['motion_seconds'])
    parser.add_argument('--range', dest='radio_range', type=float, default=config['communication_range_metres'])
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600 or not 10 <= args.writer_seconds <= 600 or not 2 <= args.radio_range <= 30:
        raise ValueError('Motion seconds must be 10 to 600; range must be 2 to 30 metres')
    from check_step3 import structural_check
    structural_check()
    extra_message_types()
    print('STEP 3: black writer first; gray robot waits outside.\n'
          'After the writer returns: gray robot visits orange and white beacons.\n'
          'Orange: throw one white ball, then extinguish that fire.\n'
          'White: left/alive turns green; right/dead turns red, on the same tile.\n'
          'Blue/yellow: avoid and leave in place. Both robots finish outside.', flush=True)
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    gazebo = controller = None
    try:
        with (logs/'gazebo-step3.log').open('w') as output:
            command = ['gz', 'sim', '-r', '-v', '3']
            if args.headless:
                command.append('-s')
            command.append(str(PACKAGE/'worlds/warehouse_step3.sdf'))
            gazebo = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            controller = subprocess.Popen([sys.executable, '-B', '-u', str(PACKAGE/'scripts/animate_step3.py'),
                                           '--writer-seconds', str(args.writer_seconds),
                                           '--seconds', str(args.seconds), '--range', str(args.radio_range)],
                                          start_new_session=True)
            while gazebo.poll() is None and controller.poll() is None:
                time.sleep(.15)
            if gazebo.poll() is not None and controller.poll() is None:
                stop_process(controller)
                return gazebo.returncode or 130
            if controller.returncode:
                print('Scenario stopped. Inspect logs/step3_result.json and logs/gazebo-step3.log.',
                      file=sys.stderr)
                return controller.returncode
            if args.headless:
                return 0
            print('Scenario finished. Fires are off; victim tiles are green/red; all beacons remain.\n'
                  'Close Gazebo or press Ctrl+C when finished.', flush=True)
            gazebo.wait()
            return gazebo.returncode
    except KeyboardInterrupt:
        print('Closing this scenario.', flush=True)
        return 130
    finally:
        stop_process(controller)
        stop_process(gazebo)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('STEP 3 LAUNCH ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)
