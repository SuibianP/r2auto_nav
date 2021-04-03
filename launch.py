#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    turtlebot3_cartographer_prefix = get_package_share_directory(
            'turtlebot3_cartographer'
            )
    cartographer_config_dir = LaunchConfiguration(
            'cartographer_config_dir',
            default=os.path.join(turtlebot3_cartographer_prefix, 'config')
            )
    configuration_basename = LaunchConfiguration(
            'configuration_basename',
            default='turtlebot3_lds_2d.lua'
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'
            ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'
            ),
        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            node_name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename]
            ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
            )
        ])
