#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Simple launch file for PX4 LQR Controller Node
    """
    
    # Declare launch argument for sim time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # LQR Controller Node
    lqr_controller_node = Node(
        package='tvc_controller',  # Replace with your actual package name
        executable='lqr_px4_controller',  # Replace with your executable name
        name='lqr_px4_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        lqr_controller_node,
    ])
