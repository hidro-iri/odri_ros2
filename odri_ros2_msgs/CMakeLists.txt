cmake_minimum_required(VERSION 3.8)
project(odri_ros2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "interfaces/msg/MotorState.msg"
  "interfaces/msg/DriverState.msg"
  "interfaces/msg/MasterBoardState.msg"
  "interfaces/msg/MotorCommand.msg"
  "interfaces/msg/DriverCommand.msg"
  "interfaces/msg/MasterBoardCommand.msg"
  "interfaces/msg/RobotCommand.msg"
  "interfaces/msg/RobotState.msg"
  "interfaces/msg/RobotFullState.msg"
  "interfaces/msg/StateCommand.msg"
  "interfaces/srv/TransitionCommand.srv"
  "interfaces/srv/DirectCommand.srv"
  "interfaces/srv/PositionCommand.srv"
  DEPENDENCIES
  std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
