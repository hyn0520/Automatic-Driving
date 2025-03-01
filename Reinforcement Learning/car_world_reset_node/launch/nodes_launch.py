from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_world_reset_node',
            executable='reset_node',
            name='reset_node',
            output='screen'
        ),
        Node(
            package='car_world_reset_node',
            executable='position_node',
            name='position_node',
            output='screen'
        )
    ])