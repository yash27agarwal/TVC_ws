import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for PX4 LQR Controller Node with YAML parameters
    """
    
    # Get package directory
    package_name = 'tvc_controller'
    package_dir = get_package_share_directory(package_name)
    
    # Path to the default config file
    default_config_file = os.path.join(package_dir, 'config', 'tvc_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the YAML configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # LQR Controller Node
    lqr_controller_node = Node(
        package=package_name,
        executable='lqr_px4_controller',
        name='lqr_px4_controller',
        output='screen',
        parameters=[
            config_file,  # Load YAML parameters
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        lqr_controller_node,
    ])