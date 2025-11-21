from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directory
    pkg_share = get_package_share_directory('flatten_pointcloud')
    
    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'flatten_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Path to flatten node parameters file'
    )
    
    # Flatten node
    flatten_node = Node(
        package='flatten_pointcloud',
        executable='flatten_node',
        name='flatten_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any additional remappings here if needed
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        flatten_node
    ])