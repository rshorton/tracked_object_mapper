from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracked_object_mapper',
            executable='tracked_object_mapper'
        )
    ])