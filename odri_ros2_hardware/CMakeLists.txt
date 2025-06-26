cmake_minimum_required(VERSION 3.8)
project(odri_ros2_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hidro_ros2_utils REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(odri_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# ---- NODE: ROBOT INTERFACE ----
list(APPEND ROBOT_INTERFACE_SRC src/robot_interface.cpp)

set(ROBOT_INTERFACE_EXEC_NAME robot_interface)

add_executable(${ROBOT_INTERFACE_EXEC_NAME} ${ROBOT_INTERFACE_SRC})

target_include_directories(
  ${ROBOT_INTERFACE_EXEC_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${ROBOT_INTERFACE_EXEC_NAME} hidro_ros2_utils)
ament_target_dependencies(${ROBOT_INTERFACE_EXEC_NAME} odri_ros2_interfaces)
ament_target_dependencies(${ROBOT_INTERFACE_EXEC_NAME} rclcpp)
target_link_libraries(${ROBOT_INTERFACE_EXEC_NAME} odri_control_interface::odri_control_interface)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${ROBOT_INTERFACE_EXEC_NAME})

# ---- INSTALL TARGETS ----

# Install folders
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install targets
install(
  TARGETS ${${PROJECT_NAME}_INSTALL_TARGETS}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

ament_package()
