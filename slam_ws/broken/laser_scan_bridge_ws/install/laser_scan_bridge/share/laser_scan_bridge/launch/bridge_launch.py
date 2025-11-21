from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch.conditions

def generate_launch_description():
    # Find package share directory
    pkg_share = FindPackageShare('laser_scan_bridge')
    
    # Path to SLAM Toolbox config
    slam_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'slam_config.yaml'
    ])
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/scan',
        description='Input scan topic from Go2 (Foxy)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/scan_bridge',
        description='Output scan topic for Nav2 (Humble)'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='Queue size for subscriber and publisher'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to launch SLAM Toolbox for mapping'
    )
    
    # Bridge node (subscriber + republisher)
    bridge_node = Node(
        package='laser_scan_bridge',
        executable='scan_subscriber_node',
        name='scan_bridge',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'queue_size': LaunchConfiguration('queue_size')
        }]
    )
    
    # SLAM Toolbox node for 2D occupancy grid mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': False}
        ],
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('use_slam')
        )
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        queue_size_arg,
        use_slam_arg,
        bridge_node,
        slam_toolbox_node
    ])