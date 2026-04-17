# ---- PACKAGE SETTING ----
cmake_minimum_required(VERSION 3.8)
project(odri_ros2_gazebo)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(odri_ros2_interfaces REQUIRED)
find_package(hidro_ros2_utils REQUIRED)
find_package(hidro_utils REQUIRED)


# ---- LIBRARY: ODRI_GAZEBO_PLUGIN ----
set(ODRI_ROS2_GAZEBO_PLUGIN_NAME odri_gazebo_plugin)

list(APPEND ODRI_ROS2_GAZEBO_PLUGIN_SRC src/odri_gazebo_plugin.cpp)

add_library(${ODRI_ROS2_GAZEBO_PLUGIN_NAME} SHARED
            ${ODRI_ROS2_GAZEBO_PLUGIN_SRC})

target_include_directories(
  ${ODRI_ROS2_GAZEBO_PLUGIN_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${ODRI_ROS2_GAZEBO_PLUGIN_NAME} rclcpp)
ament_target_dependencies(${ODRI_ROS2_GAZEBO_PLUGIN_NAME} odri_ros2_interfaces)
ament_target_dependencies(${ODRI_ROS2_GAZEBO_PLUGIN_NAME} hidro_ros2_utils)
ament_target_dependencies(${ODRI_ROS2_GAZEBO_PLUGIN_NAME} hidro_utils)

target_link_libraries(${ODRI_ROS2_GAZEBO_PLUGIN_NAME}
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

list(APPEND ${PROJECT_NAME}_INSTALL_TARGETS ${ODRI_ROS2_GAZEBO_PLUGIN_NAME})

# ---- INSTALL TARGETS ----
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gz-sim8)
ament_export_dependencies(odri_ros2_interfaces)

# Install folders
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install(
  TARGETS ${${PROJECT_NAME}_INSTALL_TARGETS}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

ament_environment_hooks(
  "env-hooks/${PROJECT_NAME}.dsv.in"
  "env-hooks/${PROJECT_NAME}.sh.in"
)

ament_package()
