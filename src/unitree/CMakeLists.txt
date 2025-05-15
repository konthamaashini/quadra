cmake_minimum_required(VERSION 3.8)
project(unitree)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)             # For Python scripts (teleop.py, joint_control_gui.py)
find_package(geometry_msgs REQUIRED)     # For Twist and Pose messages
find_package(sensor_msgs REQUIRED)       # For Joy and JointState messages
find_package(trajectory_msgs REQUIRED)   # For JointTrajectory messages

# Install Python scripts
install(PROGRAMS
  scripts/teleop.py
  scripts/joint_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  urdf
  config
  mesh
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install xacro files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xacro"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()