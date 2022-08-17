
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            node_name='my_turtle',
            output='screen'),
        Node(
            package='turtlesim',
            node_executable='draw_square',
            node_namespace='draw_square',
            output='log'),
        Node(
            package='turtlesim',
            node_executable='draw_square',
            node_name='draw_square',
            node_namespace='new_turtle',
            output='log'),
    ])