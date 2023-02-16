from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            name="trajectory_sampler_node",
            package="mav_trajectory_generation_ros",
            executable="trajectory_sampler_node",
            output="screen",
            emulate_tty=True
        )
    ])