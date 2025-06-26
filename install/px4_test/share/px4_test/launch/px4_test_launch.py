from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the px4.yaml file
    config_file = os.path.join(
        get_package_share_directory('px4_test'),
        'config',
        'px4.yaml'
    )
    print(config_file)
    # Return the LaunchDescription object with the node
    return LaunchDescription([
        Node(
            package='px4_test',
            executable='px4_test',
            name='px4_test',
            parameters=[config_file],  # Load parameters from the YAML file
            output='screen'
        )
    ])