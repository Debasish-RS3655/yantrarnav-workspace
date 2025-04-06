#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Use the absolute path for your custom MAVROS configuration file.
    custom_config_file = "/home/nits/mavros_config/custom_mavros_config.yaml"
    
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            output='screen',
            parameters=[
                custom_config_file,
                {'use_sim_time': False}  # Ensure consistent time source; set to True if using simulation time.
            ]
        ),
        # If you want to remap topics (for example, forwarding vision data)
        # You can add additional Node definitions here, e.g., a vision relay node.
    ])

if __name__ == '__main__':
    generate_launch_description()
