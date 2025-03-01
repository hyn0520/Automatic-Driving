cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(avoid_wall)

# Find the Ignition_Transport library
find_package(ignition-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node.cc")
  add_executable(lidar_node lidar_node.cc)
  target_link_libraries(lidar_node ignition-transport${IGN_TRANSPORT_VER}::core)
endif()

if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node_new.cc")
  add_executable(lidar_node_new lidar_node_new.cc)
  target_link_libraries(lidar_node_new ignition-transport${IGN_TRANSPORT_VER}::core)
endif()
