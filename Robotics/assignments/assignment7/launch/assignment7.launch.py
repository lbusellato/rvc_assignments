from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package="assignment7",
        executable="main",
    )
    ld.add_action(node)
    return ld
