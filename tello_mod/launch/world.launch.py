#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    pkg_dir = get_package_share_directory('tello_mod') 
    world_file_name = 'labor_worlds/' + 'rondell' + '.world'
    world = os.path.join(pkg_dir, 'worlds', world_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    HOME = os.environ.get('HOME')
    model = os.path.join(pkg_dir, 'models', 'iris_fpv_lidar', 'iris_fpv_lidar.sdf')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'),
            ),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='drone_spawner',
            output='screen',
            arguments=['-file', model, '-entity', 'iris'],
            ),
        ExecuteProcess(
            cmd=[
                'sudo',
                '-E',
                'bin/px4',
                '-s',
                'etc/init.d-posix/rcS',
                ],
            cwd=HOME + '/PX4-Autopilot/build/px4_sitl_rtps',
            output='screen'),
        Node(
            package='gate_spawner',
            executable='spawner',
            name='gate_spawner',
            output='screen',
            ),
        ExecuteProcess(
            cmd=['micrortps_agent', '-t', 'UDP'],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'laser_link']),
    ])

