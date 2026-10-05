"""Optional ROS 2 launch for the same complete standalone demo."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    share = Path(get_package_share_directory('factory_environment_simulation'))
    return LaunchDescription([
        ExecuteProcess(cmd=['bash', str(share/'run.sh'), 'demo'], output='screen'),
    ])
