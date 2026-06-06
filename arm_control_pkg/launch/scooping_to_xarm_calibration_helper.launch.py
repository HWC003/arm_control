from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('arm_control_pkg')
    default_config = os.path.join(package_share, 'config', 'scooping_to_xarm_calibration_helper.yaml')

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='arm_control_pkg',
            executable='scooping_to_xarm_calibration_helper',
            name='scooping_to_xarm_calibration_helper',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
        ),
    ])
