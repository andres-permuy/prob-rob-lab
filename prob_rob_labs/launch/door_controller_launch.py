"""Launch the Lab 2 controller against an already running simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from door_controller.sequence import PARAMETERS


def generate_launch_description():
    arguments = [DeclareLaunchArgument('use_sim_time', default_value='true')]
    parameters = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }
    for name, (default, description) in PARAMETERS.items():
        arguments.append(DeclareLaunchArgument(
            name, default_value=str(default), description=description))
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    return LaunchDescription(arguments + [Node(
        package='prob_rob_labs', executable='door_controller',
        name='door_controller', output='screen', parameters=[parameters],
    )])
