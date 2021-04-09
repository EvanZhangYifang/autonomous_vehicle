from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tele_operation',
            namespace='tele_operation1',
            executable='user_interface',
            name='user_interface',
            output='screen'),
        Node(
            package='tele_operation',
            namespace='tele_operation1',
            executable='random_position_server',
            name='random_position_server'),
    ])