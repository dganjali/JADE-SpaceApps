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

# Advanced orbital controller (with sensor models and thruster allocation)
find_package(Eigen3 REQUIRED)
add_library(advanced_orbital_controller SHARED src/advanced_orbital_controller.cpp)
target_link_libraries(advanced_orbital_controller gz-sim::gz-sim Eigen3::Eigen)
target_include_directories(advanced_orbital_controller PRIVATE ${EIGEN3_INCLUDE_DIR})

# Docking dashboard GUI plugin (TODO: Fix Qt widget API issues)
# find_package(gz-gui REQUIRED)
# set(CMAKE_AUTOMOC ON)
# add_library(docking_dashboard SHARED src/docking_dashboard.cpp)
# target_link_libraries(docking_dashboard gz-gui::gz-gui)

# Install all plugins to lib
install(TARGETS thruster_controller realistic_orbital_controller vision_based_orbital_controller advanced_orbital_controller
  LIBRARY DESTINATION lib
)
