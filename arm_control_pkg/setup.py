from setuptools import find_packages, setup

package_name = 'arm_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_control.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arm_control_lite6.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arm2_scooping_grasp.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arm2_extrinsic_calibration_helper.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arm2_tag_grasp_calibration_helper.launch.py']),
        ('share/' + package_name + '/launch', ['launch/scooping_to_xarm_calibration_helper.launch.py']),
        ('share/' + package_name + '/config', ['config/arm_control.yaml']),
        ('share/' + package_name + '/config', ['config/arm_control_lite6.yaml']),
        ('share/' + package_name + '/config', ['config/arm2_scooping_grasp.yaml']),
        ('share/' + package_name + '/config', ['config/arm2_extrinsic_calibration_helper.yaml']),
        ('share/' + package_name + '/config', ['config/arm2_tag_grasp_calibration_helper.yaml']),
        ('share/' + package_name + '/config', ['config/scooping_to_xarm_calibration_helper.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HWC003',
    maintainer_email='zufoichang@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_controller = arm_control_pkg.arm_controller:main',
            'arm2_scooping_grasp = arm_control_pkg.arm2_scooping_grasp:main',
            'arm2_extrinsic_calibration_helper = arm_control_pkg.arm2_extrinsic_calibration_helper:main',
            'arm2_tag_grasp_calibration_helper = arm_control_pkg.arm2_tag_grasp_calibration_helper:main',
            'lite6_xarm_tf_composer = arm_control_pkg.lite6_xarm_tf_composer:main',
            'scooping_to_xarm_calibration_helper = arm_control_pkg.scooping_to_xarm_calibration_helper:main',
            'arm_control_old_exec = arm_control_pkg.arm_control_old:main',
            'my_service_server = arm_control_pkg.my_service_server:main',
            'my_service_client = arm_control_pkg.my_service_client:main',
        ],
    },
)
