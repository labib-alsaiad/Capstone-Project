from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_pkg',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),
        Node(
            package='motor_pkg',
            executable='motor_node',
            name='motor_node',
            output='screen',
        ),
    ])
