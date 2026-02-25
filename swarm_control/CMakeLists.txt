cmake_minimum_required(VERSION 3.10)
project(swarm_control LANGUAGES NONE)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable" FORCE)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS "${CMAKE_EXPORT_COMPILE_COMMANDS}" CACHE BOOL
    "Enable generation of compile_commands.json (only meaningful for C/C++ targets)." FORCE)
endif()

# Install the Python package
ament_python_install_package(${PROJECT_NAME})

# Register this package in the ament index
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install non-Python assets
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE
)

# Generate executable entry-point wrappers (same pattern as tof_slam_sim)
set(_python_nodes
  "swarm_control.frontier_server:main:frontier_server"
  "swarm_control.goal_allocator:main:goal_allocator"
  "swarm_control.drone_executor:main:drone_executor"
  "swarm_control.mission_supervisor:main:mission_supervisor"
  "swarm_control.traffic_manager:main:traffic_manager"
  "swarm_control.swarm_dashboard:main:swarm_dashboard"
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