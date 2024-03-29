cmake_minimum_required(VERSION 3.8)
project(odri_ros2_examples)

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
find_package(Eigen3 REQUIRED)
find_package(hidro_ros2_utils REQUIRED)
find_package(odri_ros2_interfaces REQUIRED)

add_executable(example_robot src/example_robot.cpp)

ament_target_dependencies(example_robot hidro_ros2_utils)
ament_target_dependencies(example_robot odri_ros2_interfaces)
ament_target_dependencies(example_robot rclcpp)

target_include_directories(
  example_robot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR})
target_compile_features(example_robot PUBLIC c_std_99 cxx_std_14) # Require C99

# and C++14
install(TARGETS example_robot DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS example_robot DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
