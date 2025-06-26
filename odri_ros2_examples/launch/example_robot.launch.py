import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    sim_arg = DeclareLaunchArgument(
        'sim',
        description="Set to 'true' if you want to simulate the robot. 'False' if you work with real hardware",
        default_value='True')

    ld.add_action(sim_arg)

    # REAL HARDWARE
    odri_hardware_config = os.path.join(get_package_share_directory('odri_ros2_hardware'), 'config', 'params.yaml')
    odri_hardware_node = Node(package='odri_ros2_hardware',
                              name='odri_ros2_hardware',
                              executable='robot_interface',
                              parameters=[odri_hardware_config],
                              output='screen',
                              condition=UnlessCondition(LaunchConfiguration('sim')),
                              remappings=[("robot_state", "/odri/robot_state"),
                                          ("robot_command", "/odri/robot_command")])

    # GAZEBO
    gazebo_path = PathJoinSubstitution([FindPackageShare('hidro_robots'), 'launch', '_gazebo.launch.py'])
    gazebo_src = PythonLaunchDescriptionSource([gazebo_path])
    gazebo_launch = IncludeLaunchDescription(gazebo_src,
                                             launch_arguments={"robot_name": "flying_arm_2"}.items(),
                                             condition=IfCondition(LaunchConfiguration('sim')))
    ld.add_action(gazebo_launch)

    # ROBOT EXAMPLE NODE
    odri_example_node = Node(
        package='odri_ros2_examples',
        name='example_robot',
        executable='example_robot',
        output='screen',
        #  remappings=[("robot_state", "/odri/robot_state"),
        #              ("robot_command", "/odri/robot_command")]
        remappings=[("robot_state_transition", "/odri_gazebo/state_transition")])

    reconfigure = Node(package='rqt_reconfigure', name='rqt_reconfigure', executable='rqt_reconfigure')

    ld.add_action(odri_hardware_node)
    ld.add_action(odri_example_node)
    ld.add_action(reconfigure)

    return ld