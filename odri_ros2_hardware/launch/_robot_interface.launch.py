from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    ld = LaunchDescription()

    robot_name_arg = DeclareLaunchArgument('robot_name',
                                           description='Name of the robot controlled by the ODRI master board.')
    yaml_path_arg = DeclareLaunchArgument('yaml_path',
                                          description='Path of the yaml file with state publisher node parameters')

    remappings = [('robot_state', '/odri/robot_state')]

    node = Node(
        package='odri_ros2_hardware',
        name='robot_interface',
        executable='robot_interface',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('yaml_path')],
        remappings=remappings,
        namespace='odri',
        prefix=[  # Sudo command cause need to be sudoer when we do this node cause it real time
            "sudo -E env PATH=",
            EnvironmentVariable("PATH", default_value="${PATH}"),
            " LD_LIBRARY_PATH=",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
            " PYTHONPATH=",
            EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
            " HOME=/tmp ",
        ],
    )

    ld.add_action(robot_name_arg)
    ld.add_action(yaml_path_arg)
    ld.add_action(node)

    return ld