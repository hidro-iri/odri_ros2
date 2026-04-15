"""
Solo 12 — ODRI ROS2 Gazebo simulation launch file.

Starts Gazebo with the Solo 12 quadruped model loaded with the ODRI
Gazebo plugin.  The plugin mirrors the real hardware interface:

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
  ros2 launch odri_ros2_examples solo12_gazebo.launch.py event_based_sim:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # ── Arguments ──────────────────────────────────────────────────────────
    ld.add_action(
        DeclareLaunchArgument(
            'event_based_sim',
            default_value='False',
            description='Use event-based Gazebo physics (disables GUI)'))

    # ── World file ─────────────────────────────────────────────────────────
    world_filename = PythonExpression([
        "'event_based.world' if ",
        LaunchConfiguration('event_based_sim'),
        " else 'basic_world.world'",
    ])
    world_path = PathJoinSubstitution(
        [FindPackageShare('hidro_robots'), 'worlds', world_filename])

    enable_gui = PythonExpression([
        "'false' if ",
        LaunchConfiguration('event_based_sim'),
        " else 'true'",
    ])

    # ── Gazebo ─────────────────────────────────────────────────────────────
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={
                'verbose':   'false',
                'pause':     'false',
                'world':     world_path,
                'lockstep':  'true',
                'gui':       enable_gui,
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
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── Robot state publisher (in 'gazebo' namespace, matches _gazebo.launch) ──
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='gazebo',
            output='screen',
            parameters=[robot_description]))

    # ── Spawn Solo 12 in Gazebo ────────────────────────────────────────────
    # z=0.4 gives the legs clearance above ground at startup (joints at 0).
    # Leg reach with all joints at 0: upper(0.16) + lower(0.16) = 0.32 m
    # → feet land at z = 0.4 − 0.32 = 0.08 m above ground before settling.
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'gazebo/robot_description',
                '-entity', 'solo12',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.4',
            ],
            output='screen'))

    return ld
