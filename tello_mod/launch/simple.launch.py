"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'worlds','labor_worlds', 'rondell.world')  #/home/tom/robot_ws/src/tello_mod/worlds/labor_worlds/rondell.world
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'models')),
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1', '0']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             namespace=ns),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='Tell_Mod', executable='tello_teleop.py', output='screen',
             namespace=ns),
    ])
