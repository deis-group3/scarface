"""Launch file for Scarface robot fish node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for Scarface."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # Create the Scarface node
    scarface_node = Node(
        package='scarface',
        executable='scarface_node',
        name='scarface_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        scarface_node,
    ])
