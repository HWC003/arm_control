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
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/config', ['config/defaults.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HWC003',
    maintainer_email='zufoichang@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_ros2_node_exec = arm_control_pkg.my_ros2_node:main',
            'my_service_server = arm_control_pkg.my_service_server:main',
            'my_service_client = arm_control_pkg.my_service_client:main',
        ],
    },
)
