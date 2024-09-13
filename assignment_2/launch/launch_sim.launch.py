import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='assignment_2'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    spawn_entity = Node(package='assignment_2', executable='spawn_entity',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot', '-x', '1.0', '-y', '0.0', '-z', '0.0'],
                        output='screen') 
    
    steering_gazebo = Node(package='assignment_2', executable='steering_gazebo',
                        output='screen')

    # Launch them all!
    return LaunchDescription([
        rsp,
        # gazebo,
        # spawn_entity,
        steering_gazebo,
    ])