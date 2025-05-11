cmake_minimum_required(VERSION 3.8)
project(unitree)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)         # Added for teleop.py
find_package(geometry_msgs REQUIRED) # Added for Twist and Pose messages
find_package(sensor_msgs REQUIRED)   # Added for Joy messages

# Optional: Add compiler flags (useful if you add C++ code later)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install Python scripts
install(PROGRAMS
  scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (URDF, mesh, worlds, launch)
install(
  DIRECTORY urdf mesh worlds launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (remove these when licenses are added)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()