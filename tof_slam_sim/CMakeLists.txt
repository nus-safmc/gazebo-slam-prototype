cmake_minimum_required(VERSION 3.10)
project(tof_slam_sim LANGUAGES NONE)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable" FORCE)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Honour external requests for compile command exports to avoid CMake warnings.
if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS "${CMAKE_EXPORT_COMPILE_COMMANDS}" CACHE BOOL
    "Enable generation of compile_commands.json (only meaningful for C/C++ targets)." FORCE)
endif()

# Install the Python package (the folder "tof_slam_sim" with __init__.py)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/tof8x8_to_scan.py
  src/scan_merger.py
  src/test_controller.py
  src/tof_to_scan.py
  src/pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
# Register this package in the ament index
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install non-Python assets if they exist
install(DIRECTORY
  launch
  config
  scripts
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE
)

set(_python_nodes
  "tof_slam_sim.scan_merger:main:scan_merger"
  "tof_slam_sim.auto_pilot_node:main:auto_pilot"
  "tof_slam_sim.odom_tf_publisher:main:odom_tf_publisher"
  "tof_slam_sim.swarm_tf_broadcaster:main:swarm_tf_broadcaster"
  "tof_slam_sim.map_tf_fallback:main:map_tf_fallback"
  "tof_slam_sim.swarm_map_fuser:main:swarm_map_fuser"
  "tof_slam_sim.tf_namespace_relay:main:tf_namespace_relay"
  "tof_slam_sim.cmd_vel_relay:main:cmd_vel_relay"
  "tof_slam_sim.nav2_frontier_explorer:main:nav2_frontier_explorer"
  "tof_slam_sim.topic_monitor:main:topic_monitor"
  "tof_slam_sim.drone_health_dashboard:main:drone_health_dashboard"
)

foreach(entry IN LISTS _python_nodes)
  string(REPLACE ":" ";" parts "${entry}")
  list(GET parts 0 _module)
  list(GET parts 1 _function)
  list(GET parts 2 _target)
  set(PYTHON_ENTRYPOINT_MODULE "${_module}")
  set(PYTHON_ENTRYPOINT_FUNCTION "${_function}")
  configure_file(
    cmake/python_entrypoint.in
    "${CMAKE_CURRENT_BINARY_DIR}/${_target}"
    @ONLY
    FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
  install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${_target}"
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
