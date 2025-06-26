# ODRI ROS2 Package for msgs & srv

Collection of ROS2 messages and services

> In construction :construction_worker:

## Messages

- [MotorCommand](interfaces/msg/MotorCommand.msg) : Command send to a motor
- [MotorState](interfaces/msg/MotorState.msg) : Current state of a motor
- [DriverCommand](interfaces/msg/DriverCommand.msg) : Command send to the udriver
- [DriverState](interfaces/msg/DriverState.msg) : Current state of the udriver
- [MasterBoardCommand](interfaces/msg/MasterBoardCommand.msg) : Command send to the master-board
- [MasterBoardState](interfaces/msg/MasterBoardState.msg) : Current state of the master-board
- [RobotCommand](interfaces/msg/RobotCommand.msg) : Command send to the robot
- [RobotState](interfaces/msg/RobotState.msg) : Current state of the robot
- [RobotFullState](interfaces/msg/RobotFullState.msg) : Current state of the robot + state of the finite state machine
- [StateCommand](interfaces/msg/StateCommand.msg) : Current state of the finite state machine

## Services

- [TransitionCommand](interfaces/srv/TransitionCommand.srv) : Request change of the finite state machine
