from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('arm_control_pkg'),
        'config', 
        'arm_control.yaml')
    
    return LaunchDescription([
        Node(
            package='arm_control_pkg',
            executable='arm_controller',
            name='arm_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],

        ),
    ])
