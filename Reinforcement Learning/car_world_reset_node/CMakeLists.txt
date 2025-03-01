cmake_minimum_required(VERSION 3.8)
project(car_world_reset_node)

# 1) ROS 2 ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)  # for Trigger.srv

# 2) Ignition Transport  (Fortress ~ ignition-transport11)
set(CMAKE_PREFIX_PATH "/home/yinan/ign-transport/build/install:${CMAKE_PREFIX_PATH}")
find_package(ignition-transport11 REQUIRED)
find_package(geometry_msgs REQUIRED)
# 3) C++14/17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4) 包含头文件
include_directories(
  include
)

add_executable(reset_node
  src/reset_node.cpp

  src/reset_node_main.cpp  # 如果你拆分一个 main 另存，可在这里加
)
ament_target_dependencies(reset_node
  rclcpp
  std_srvs
)

# 5) 可执行文件

add_executable(position_node
  src/position_node.cpp

  src/position_node_main.cpp  # 如果你拆分一个 main 另存，可在这里加
)
ament_target_dependencies(position_node
  rclcpp
  geometry_msgs
)




target_link_libraries(reset_node
  ignition-transport11::ignition-transport11
)

target_link_libraries(position_node
  ignition-transport11::ignition-transport11
)

# 7) 安装可执行文件
install(TARGETS reset_node position_node
  DESTINATION lib/${PROJECT_NAME}
)

# 在 ament_package() 前添加
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 8) ament_package
ament_package()

