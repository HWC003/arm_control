from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('arm_control_pkg')
    default_config = os.path.join(package_share, 'config', 'arm2_extrinsic_calibration_helper.yaml')

    config_file = LaunchConfiguration('config_file')
    show_rviz = LaunchConfiguration('show_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('show_rviz', default_value='false'),
        Node(
            package='arm_control_pkg',
            executable='arm2_extrinsic_calibration_helper',
            name='arm2_extrinsic_calibration_helper',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_arm2_calibration',
            output='screen',
            condition=IfCondition(show_rviz),
        ),
    ])
