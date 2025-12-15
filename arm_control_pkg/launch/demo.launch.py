from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_ros_python_pkg')
    params = os.path.join(pkg, 'config', 'defaults.yaml')
    return LaunchDescription([
        Node(
            package='my_ros_python_pkg',
            executable='my_ros2_node_exec',
            name='my_python_node',
            parameters=[params],
            output='screen',
        ),
    ])
