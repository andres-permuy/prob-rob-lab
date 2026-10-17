"""Launch the Lab 3 controller against an already running simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PARAMETERS = {
    'feature_threshold': 287.5,
    'prior_open': 0.5,
    'p_z_open_given_open': 0.95,
    'p_z_open_given_closed': 0.05,
    'open_command_success': 0.2,
    'decision_probability': 0.99,
    'measurement_period': 0.5,
    'command_period': 3.5,
    'startup_delay': 1.0,
    'forward_speed': 0.2,
    'drive_duration': 22.0,
    'stop_duration': 1.0,
    'close_duration': 8.0,
    'open_torque': 5.0,
    'close_torque': -5.0,
}


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='threshold'),
    ]
    parameters = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
        'mode': LaunchConfiguration('mode'),
    }
    for name, default in PARAMETERS.items():
        arguments.append(DeclareLaunchArgument(name, default_value=str(default)))
        parameters[name] = ParameterValue(
            LaunchConfiguration(name), value_type=float)

    controller = Node(
        package='prob_rob_labs',
        executable='lab3_controller',
        name='lab3_controller',
        output='screen',
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [controller])
