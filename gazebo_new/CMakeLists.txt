cmake_minimum_required(VERSION 3.10)
project(orbital_controllers)

# Find gz-sim (Gazebo)
find_package(gz-sim REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Original thruster controller
add_library(thruster_controller SHARED src/thruster_controller.cpp)
target_link_libraries(thruster_controller gz-sim::gz-sim)

# Realistic orbital controller
add_library(realistic_orbital_controller SHARED src/realistic_orbital_controller.cpp)
target_link_libraries(realistic_orbital_controller gz-sim::gz-sim)

# Vision-based orbital controller
add_library(vision_based_orbital_controller SHARED src/vision_based_orbital_controller.cpp)
target_link_libraries(vision_based_orbital_controller gz-sim::gz-sim)

# Install all plugins to lib
install(TARGETS thruster_controller realistic_orbital_controller vision_based_orbital_controller
  LIBRARY DESTINATION lib
)
