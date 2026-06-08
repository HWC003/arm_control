from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('arm_control_pkg')
    default_config = os.path.join(package_share, 'config', 'arm2_scooping_grasp.yaml')

    config_file = LaunchConfiguration('config_file')
    publish_camera_to_arm2_tf = LaunchConfiguration('publish_camera_to_arm2_tf')
    camera_source_frame = LaunchConfiguration('camera_source_frame')
    arm2_base_frame = LaunchConfiguration('arm2_base_frame')

    tx = LaunchConfiguration('camera_to_arm2_tx')
    ty = LaunchConfiguration('camera_to_arm2_ty')
    tz = LaunchConfiguration('camera_to_arm2_tz')
    rr = LaunchConfiguration('camera_to_arm2_roll')
    rp = LaunchConfiguration('camera_to_arm2_pitch')
    ry = LaunchConfiguration('camera_to_arm2_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('publish_camera_to_arm2_tf', default_value='true'),
        DeclareLaunchArgument('camera_source_frame', default_value='orbbec_camera_color_optical_frame'),
        DeclareLaunchArgument('arm2_base_frame', default_value='xarm2_base'),
        DeclareLaunchArgument('camera_to_arm2_tx', default_value='0.132290'),
        DeclareLaunchArgument('camera_to_arm2_ty', default_value='-0.294686'),
        DeclareLaunchArgument('camera_to_arm2_tz', default_value='0.121022'),
        DeclareLaunchArgument('camera_to_arm2_roll', default_value='-2.407647'),
        DeclareLaunchArgument('camera_to_arm2_pitch', default_value='0.790730'),
        DeclareLaunchArgument('camera_to_arm2_yaw', default_value='-1.565212'),

        Node(
            package='arm_control_pkg',
            executable='arm2_scooping_grasp',
            name='arm2_scooping_grasp',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arm2_camera_static_tf',
            condition=IfCondition(publish_camera_to_arm2_tf),
            arguments=[
                '--x',
                tx,
                '--y',
                ty,
                '--z',
                tz,
                '--roll',
                rr,
                '--pitch',
                rp,
                '--yaw',
                ry,
                '--frame-id',
                camera_source_frame,
                '--child-frame-id',
                arm2_base_frame,
            ],
        ),
    ])
