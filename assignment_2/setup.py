from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'assignment_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'robot.urdf.xacro'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'robot_core.xacro'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'inertial_macros.xacro'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', 'gazebo_control.xacro'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'launch_sim.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'rsp.launch.py'))),
    ],
    install_requires=['setuptools','rclpy','matplotlib'],
    zip_safe=True,
    maintainer='kingayush',
    maintainer_email='ayush.prasad0511@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'steering='+package_name+'.steering_control:main',
                             'spawn_entity='+package_name+'.spawn_entity:main',
                             'steering_gazebo='+package_name+'.steering_control_gazebo:main',
        ],
    },
)
