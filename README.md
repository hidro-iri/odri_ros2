# odri_ros2

ROS 2 tools to interface ODRI-based robots, developed by the [HiDRo group](https://www.iri.upc.edu/groups/34) at IRI (Institut de Robòtica i Informàtica Industrial, UPC–CSIC).

This repository provides the full software stack to control robots built around the [ODRI master-board](https://github.com/open-dynamic-robot-initiative/master-board), both on real hardware and in Gazebo simulation. It is currently used with the **Borinot** aerial manipulator (flying arm) and the **Solo12** quadruped.

---

## Repository structure

```
odri_ros2/
├── odri_ros2_interfaces/   # ROS 2 message and service definitions
├── odri_ros2_hardware/     # Hardware interface node (real robot)
├── odri_ros2_gazebo/       # Gazebo Classic simulation plugin
└── odri_ros2_examples/     # Example controller node
```

---

## Packages

### `odri_ros2_interfaces`

Defines all ROS 2 messages and services used across the stack.

**Messages:**

| Message | Description |
|---|---|
| `MotorCommand` | Per-motor command: position, velocity, torque reference, and gains Kp/Kd |
| `MotorState` | Per-motor state: position, velocity, torque, enable flag, encoder index |
| `DriverCommand` | Command to a uDriver (2 motors) |
| `DriverState` | State of a uDriver (2 motors) |
| `MasterBoardCommand` | Command to all uDrivers via the master board |
| `MasterBoardState` | State of all uDrivers |
| `RobotCommand` | High-level command: array of `MotorCommand` for all joints |
| `RobotState` | High-level state: array of `MotorState` for all joints |
| `StateCommand` | Reports the active state of the finite state machine |

`StateCommand` defines the following states:

```
IDLE (0) → CALIBRATING (1) → ENABLED (2) → RUNNING (3)
                                            OTHER / Error (4)
```

**Services:**

| Service | Description |
|---|---|
| `TransitionCommand` | Request a state machine transition by name; returns resulting state and message |
| `DirectCommand` | Send a torque command with position limits |
| `PositionCommand` | Send a position command with Kp/Kd gains |

---

### `odri_ros2_hardware`

Hardware interface node that communicates directly with the ODRI master board over Ethernet. It wraps `odri_control_interface` and exposes the robot as ROS 2 topics.

**Node:** `robot_interface` (namespace `odri`)

**Subscribes to:**
- `/odri/robot_command` (`RobotCommand`) — desired joint positions, velocities, torques, and gains

**Publishes:**
- `/odri/robot_state` (`RobotState`) — current joint positions, velocities, and torques

The node implements a **finite state machine** with the following transitions:

```
IDLE ──enable──► CALIBRATING_OFFSETS ──► CALIBRATING_SAFE_CONFIG ──► ENABLED ──start──► RUNNING
                                                                      ◄─disable──        ◄──stop──
```

During calibration the node performs two phases: encoder index search (`CALIBRATING_OFFSETS`, direction per joint as configured) and movement to the safe configuration (`CALIBRATING_SAFE_CONFIG`). A safe configuration is applied on disable.

> **Note:** The node requires `sudo` because it uses real-time Ethernet communication. The provided launch file handles this automatically by forwarding the necessary environment variables.

**Parameters** (`robot_interface` node):

| Parameter | Description |
|---|---|
| `robot_yaml_name` | Name of the ODRI robot YAML file (located in `config/robots/`) |
| `n_slaves` | Number of uDriver boards connected to the master board |
| `safe_configuration` | Joint positions to hold when disabling (radians) |
| `safe_kp` / `safe_kd` | Position/velocity gains applied during safe hold |
| `safe_torque` | Maximum torque during safe hold (Nm) |
| `safe_current` | Maximum current during safe hold (A) |

**Robot YAML** (`config/robots/<robot>.yaml`):

```yaml
robot:
    interface: <ethernet_interface>   # e.g. enxa0cec8792aa5
    joint_modules:
        motor_numbers: [0, 1]
        motor_constants: 0.025        # Nm/A
        gear_ratios: 9.0
        max_currents: 4.0             # A
        reverse_polarities: [true, true]
        lower_joint_limits: [-2.0, -3.14]  # rad
        upper_joint_limits: [+1.5, +3.14]  # rad
        max_joint_velocities: 80.0    # rad/s
        safety_damping: 0.5
    imu:
        rotate_vector: [1, 2, 3]
        orientation_vector: [1, 2, 3, 4]
joint_calibrator:
    search_methods: [POS, POS]        # POS, NEG, ALT or AUTO
    position_offsets: [0.306, 0.289]  # rad
    Kp: 0.5
    Kd: 5.0
    T: 1.0                            # calibration duration (s)
    dt: 0.001                         # control timestep (s)
```

**Launch:**

```bash
ros2 launch odri_ros2_hardware _robot_interface.launch.py \
    robot_name:=flying_arm_2 \
    yaml_path:=<path_to_params.yaml>
```

---

### `odri_ros2_gazebo`

Gazebo Classic plugin (`OdriGazeboPlugin`) that simulates the ODRI interface, exposing the same ROS 2 topics as the hardware node so that `odri_ros2_examples` and other controllers work identically in simulation and on the real robot.

**Subscribes to:**
- `/odri/robot_command` (`RobotCommand`) — joint commands (position, velocity, torque, gains)

**Publishes:**
- `/odri/robot_state` (`RobotState`) — simulated joint state (~667 Hz)
- `/odri/state_machine_status` (`StateMachineStatus`) — active state (5 Hz)

The plugin implements the IDLE → ENABLED → RUNNING state machine (no calibration phase, unlike the hardware node), and uses `hidro_ros2_utils::StateMachine` for transitions.

The state transition service uses `hidro_ros2_utils/srv/TransitionCommand`:

```bash
ros2 service call /odri/robot_interface/state_transition \
  hidro_ros2_utils/srv/TransitionCommand "{command: 'enable'}"
```

**Usage:** Include the plugin in the robot's SDF/URDF via the `hidro_robots` description package. Launch the Solo12 simulation with:

```bash
ros2 launch odri_ros2_examples solo12_gazebo.launch.py gui:=false
```

---

### `odri_ros2_examples`

A minimal example controller node (`OdriControl`) that demonstrates how to command a 2-DOF robot joint via the `odri_ros2` stack.

**Node:** `example_robot`

**Subscribes to:**
- `/odri/robot_state` (`RobotState`)

**Publishes to:**
- `/odri/robot_command` (`RobotCommand`)

**Services exposed:**
- `direct_command` (`DirectCommand`) — send a torque command with position limits
- `position_command` (`PositionCommand`) — send a position setpoint with gains

The node supports three internal control modes, selectable at runtime:

| Mode | Description |
|---|---|
| `position` | Hold a desired joint position with configurable Kp/Kd |
| `direct` | Apply a desired torque with position-based saturation |
| `transition` | Smooth polynomial trajectory between two configurations |

**Launch (simulation — flying_arm_2):**

```bash
ros2 launch odri_ros2_examples example_robot.launch.py sim:=true
```

**Launch (real hardware):**

```bash
ros2 launch odri_ros2_examples example_robot.launch.py sim:=false
```

Both launch files also start `rqt_reconfigure` to tune control parameters online.

> **Note:** The `example_robot` node is designed for the 2-DOF flying arm. For Solo12 (12 joints), use `solo12_gazebo.launch.py` and a dedicated controller.

---

## Dependencies

- ROS 2 (tested on Humble)
- [`odri_control_interface`](https://github.com/open-dynamic-robot-initiative/odri_control_interface) — low-level C++ API to the ODRI master board
- [`master_board_sdk`](https://github.com/open-dynamic-robot-initiative/master-board) — Ethernet communication with the master board
- `hidro_ros2_utils` — HiDRo state machine and utility nodes
- `hidro_robots` — robot URDF/SDF descriptions and Gazebo launch files
- Gazebo Classic (`gazebo_ros`) — for simulation
- Eigen3

---

## Build

```bash
cd ~/ros2_ws/src
git clone <this_repo>
cd ~/ros2_ws
colcon build --packages-select \
    odri_ros2_interfaces \
    odri_ros2_hardware \
    odri_ros2_gazebo \
    odri_ros2_examples
source install/setup.bash
```

---

## Architecture overview

```
┌─────────────────────────────────────────┐
│         Controller / Planner            │
│   (e.g. eagle_ros2 MPC, example_robot)  │
└────────────┬───────────────▲────────────┘
             │ /odri/robot_command │ /odri/robot_state
     ┌───────▼───────────────┴────────┐
     │   odri_ros2_hardware            │  (real)
     │   odri_ros2_gazebo plugin       │  (sim)
     └───────────────┬────────────────┘
                     │ Ethernet / Gazebo physics
             ┌───────▼────────┐
             │  master-board  │
             │  + uDrivers    │
             │  + motors      │
             └────────────────┘
```

The hardware and Gazebo nodes expose an **identical ROS 2 interface**, so any controller developed in simulation runs on the real robot without modification.

---

## Maintainer

Developed at the [HiDRo group](https://www.iri.upc.edu/groups/34), IRI (UPC–CSIC).  
Contact: `jbrotons@iri.upc.edu`
