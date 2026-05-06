"""
Solo 12 — ODRI ROS2 real hardware launch file.

Starts the robot_interface node connected to the Solo12 via the ODRI
Master Board over Ethernet. Exposes the same ROS2 interface as the
Gazebo simulation:

  Topics
    /odri/robot_state   (odri_ros2_interfaces/RobotState)   — published
    /odri/robot_command (odri_ros2_interfaces/RobotCommand)  — subscribed
    /odri/joint_states  (sensor_msgs/JointState)             — published
    /tf, /tf_static                                          — published

  Service
    /odri/robot_interface/state_transition (hidro_ros2_utils/TransitionCommand)

Workflow
  1. Connect the Master Board to the host machine via Ethernet.
  2. Set the correct Ethernet interface name in config/robots/solo12.yaml.
  3. Launch this file.
  4. Call state_transition with command="enable"
       → robot calibrates encoders and moves to safe standing configuration.
  5. Call state_transition with command="start"
       → robot accepts RobotCommand messages (your controller takes over).

Usage
  ros2 launch odri_ros2_hardware solo12_hardware.launch.py

  Override the params file:
  ros2 launch odri_ros2_hardware solo12_hardware.launch.py \\
      params_file:=/path/to/your_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # ── Arguments ──────────────────────────────────────────────────────────
    default_params = os.path.join(
        get_package_share_directory('odri_ros2_hardware'),
        'config', 'solo12_params.yaml')

    ld.add_action(
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the ROS2 parameters YAML file for the robot_interface node'))

    # ── Robot description (xacro → URDF) ──────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('hidro_robots'),
            'robots', 'solo12', 'xacro', 'description.urdf.xacro',
        ]),
        ' simulation:=false',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── robot_state_publisher ──────────────────────────────────────────────
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='odri',
            output='screen',
            parameters=[robot_description],
        ))

    # ── robot_interface node ───────────────────────────────────────────────
    # Must run as root (sudo) for real-time Ethernet communication with the
    # Master Board. The prefix forwards all necessary environment variables.
    ld.add_action(
        Node(
            package='odri_ros2_hardware',
            executable='robot_interface',
            name='robot_interface',
            namespace='odri',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('robot_state',   '/odri/robot_state'),
                ('robot_command', '/odri/robot_command'),
            ],
            prefix=[
                'sudo -E env PATH=',
                EnvironmentVariable('PATH',        default_value='${PATH}'),
                ' LD_LIBRARY_PATH=',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value='${LD_LIBRARY_PATH}'),
                ' PYTHONPATH=',
                EnvironmentVariable('PYTHONPATH',   default_value='${PYTHONPATH}'),
                ' HOME=/tmp ',
            ],
        ))

    return ld
