cmake_minimum_required(VERSION 3.10)
project(thruster_controller)

# Find gz-sim (Gazebo)
find_package(gz-sim REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(thruster_controller SHARED src/thruster_controller.cpp)

target_link_libraries(thruster_controller
  gz-sim::gz-sim
)

# Install plugin to lib
install(TARGETS thruster_controller
  LIBRARY DESTINATION lib
)
