cmake_minimum_required(VERSION 3.5)
project(simulation)

# ------------------------------------------------------------------------------
# 设置 C++ 标准
# ------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 如果编译器是 GNUC++ 或 Clang，就加一些警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# 查找依赖
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

# ------------------------------------------------------------------------------
# translator_node (可执行文件)
# ------------------------------------------------------------------------------
add_executable(translator_node
  src/translator_node_node.cpp
  src/translator_node.cpp
)

ament_target_dependencies(
  translator_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

target_include_directories(translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装 translator_node 到相应目录
install(TARGETS translator_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# SampleSystem (库 / Ignition Plugin + ROS)
# ------------------------------------------------------------------------------
add_library(SampleSystem SHARED src/TestDiffDrive.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)

target_include_directories(SampleSystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 声明对 ROS 和 Ignition 的依赖
ament_target_dependencies(
  SampleSystem
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 链接 Ignition 库
target_link_libraries(SampleSystem
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo6::ignition-gazebo6
)

# 安装 SampleSystem
install(TARGETS SampleSystem
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 安装其他资源 (如 launch 文件夹)
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 结束: ament_package 必须在最后
# ------------------------------------------------------------------------------
ament_package()
