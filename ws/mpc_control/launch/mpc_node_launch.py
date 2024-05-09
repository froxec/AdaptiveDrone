from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc_control',
            executable='mpc_node',
            namespace="control",
            name="controller"
        )
    ])