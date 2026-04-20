"""
Solo 12 — ODRI ROS2 Gazebo simulation launch file.

Starts Gazebo Harmonic with the Solo 12 quadruped model loaded with the
ODRI Gazebo plugin.  The plugin mirrors the real hardware interface:

  Topics
    /odri/robot_state   (odri_ros2_interfaces/RobotState)   — published
    /odri/robot_command (odri_ros2_interfaces/RobotCommand)  — subscribed

  Service
    /odri/robot_interface/state_transition                   — IDLE → ENABLED → RUNNING

Workflow
  1. Launch this file.
  2. Call the state_transition service with command="enable"
       → robot moves to standing safe configuration (PD, Kp=2, Kd=0.2).
  3. Call state_transition with command="start"
       → plugin accepts RobotCommand messages (your controller takes over).

Usage
  ros2 launch odri_ros2_examples solo12_gazebo.launch.py
  ros2 launch odri_ros2_examples solo12_gazebo.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                             IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Expose the odri_ros2_gazebo plugin to Gazebo Harmonic
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix('odri_ros2_gazebo'), 'lib')))

    # Expose hidro_robots meshes so model://hidro_robots/... URIs resolve
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('hidro_robots'))))

    # ── Arguments ──────────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'event_based_sim',
        default_value='False',
        description='Use faster physics step (1 kHz instead of 250 Hz)'))

    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run Gazebo headless (server only)'))

    # ── World file ─────────────────────────────────────────────────────────
    world_filename = PythonExpression([
        "'event_based.world' if ",
        LaunchConfiguration('event_based_sim'),
        " else 'basic_world.world'"])
    world_path = PathJoinSubstitution(
        [FindPackageShare('hidro_robots'), 'worlds', world_filename])

    # gz sim flags: -r = run immediately; -s = server only (no GUI)
    gz_server_flag = PythonExpression([
        "' -s' if ('",
        LaunchConfiguration('gui'),
        "' == 'false' or ",
        LaunchConfiguration('event_based_sim'),
        ") else ''"])

    # ── Gazebo Harmonic ────────────────────────────────────────────────────
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments={
                'gz_args': [world_path, ' -r', gz_server_flag],
            }.items()))

    # ── Robot description (xacro → URDF) ───────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('hidro_robots'),
            'robots', 'solo12', 'xacro', 'description.urdf.xacro',
        ]),
        ' event_based_sim:=', LaunchConfiguration('event_based_sim'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── Robot state publisher ──────────────────────────────────────────────
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='gazebo',
            output='screen',
            parameters=[robot_description]))

    # ── Spawn Solo 12 in Gazebo Harmonic ──────────────────────────────────
    # z=0.4 gives the legs clearance above ground at startup (joints at 0).
    # Leg reach with all joints at 0: upper(0.16) + lower(0.16) = 0.32 m
    # → feet land at z = 0.4 − 0.32 = 0.08 m above ground before settling.
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'gazebo/robot_description',
                '-name',  'solo12',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.4',
            ],
            output='screen'))

    return ld
