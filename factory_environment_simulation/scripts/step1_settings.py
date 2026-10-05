#!/usr/bin/env python3
"""Shared motion options: launcher and controller read the same JSON defaults."""
import json

MAX_LINEAR_SPEED = 20.0
MAX_ANGULAR_SPEED = 3.0


def load_config(package):
    return json.loads((package/'step1/config.json').read_text())


def motion_arguments(parser, config):
    parser.add_argument('--start-delay', type=float, default=config['start_delay'],
                        help='Outside wait: real seconds for animation, simulated seconds with --physics')
    parser.add_argument('--speed', type=float, default=None,
                        help=f'Maximum driving speed in m/s, 0.1 to {MAX_LINEAR_SPEED:g}')


def validate_motion(args):
    if args.start_delay < 0:
        raise ValueError('--start-delay must be zero or positive')
    if args.speed is not None and not 0.1 <= args.speed <= MAX_LINEAR_SPEED:
        raise ValueError(f'--speed must be between 0.1 and {MAX_LINEAR_SPEED:g} m/s')
