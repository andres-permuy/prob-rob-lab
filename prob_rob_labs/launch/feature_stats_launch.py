"""Launch the Assignment 5 feature statistics collector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('actual_state', default_value='closed'),
        DeclareLaunchArgument('threshold', default_value='287.5'),
        DeclareLaunchArgument('window_seconds', default_value='30.0'),
    ]
    collector = Node(
        package='prob_rob_labs',
        executable='feature_stats',
        name='feature_stats',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'actual_state': LaunchConfiguration('actual_state'),
            'threshold': ParameterValue(
                LaunchConfiguration('threshold'), value_type=float),
            'window_seconds': ParameterValue(
                LaunchConfiguration('window_seconds'), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [collector])
