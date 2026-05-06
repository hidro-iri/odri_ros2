from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    ld = LaunchDescription()

    robot_name_arg = DeclareLaunchArgument('robot_name',
                                           description='Name of the robot controlled by the ODRI master board.')
    yaml_path_arg = DeclareLaunchArgument('yaml_path',
                                          description='Path of the yaml file with state publisher node parameters')
    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        description='Path to the robot xacro/URDF file for robot_state_publisher')

    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('robot_description_file'),
                     ' simulation:=false']),
            value_type=str)}

    remappings = [('robot_state', '/odri/robot_state')]

    ld.add_action(robot_name_arg)
    ld.add_action(yaml_path_arg)
    ld.add_action(robot_description_file_arg)

    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='odri',
            output='screen',
            parameters=[robot_description],
        ))

    ld.add_action(
        Node(
            package='odri_ros2_hardware',
            name='robot_interface',
            executable='robot_interface',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('yaml_path')],
            remappings=remappings,
            namespace='odri',
            prefix=[
                "sudo -E env PATH=",
                EnvironmentVariable("PATH", default_value="${PATH}"),
                " LD_LIBRARY_PATH=",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
                " PYTHONPATH=",
                EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
                " HOME=/tmp ",
            ],
        ))

    return ld