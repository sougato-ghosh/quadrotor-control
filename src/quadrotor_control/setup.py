import os
from setuptools import find_packages, setup

package_name = 'quadrotor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml', 'world.sdf']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jules',
    maintainer_email='jules@example.com',
    description='Voice and Manual controller for X3 Quadrotor under ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = quadrotor_control.control_node:main',
        ],
    },
)
