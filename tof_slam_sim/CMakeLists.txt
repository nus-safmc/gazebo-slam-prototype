cmake_minimum_required(VERSION 3.8)
project(tof_slam_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(slam_toolbox REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/tof8x8_to_scan.py
  src/scan_merger.py
  src/test_controller.py
  scripts/auto_pilot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Provide a ROS-executable-friendly wrapper without the .py suffix so launch files
# can reference `executable="auto_pilot"`.
install(PROGRAMS scripts/auto_pilot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_pilot
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Gazebo models and worlds
install(DIRECTORY
  models
  worlds
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
