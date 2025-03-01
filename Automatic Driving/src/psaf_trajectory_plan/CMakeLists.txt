cmake_minimum_required(VERSION 3.8)
project(psaf_trajectory_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(psaf_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
#.......
# according to dependencies


include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # GoogleTest hinzufügen
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(trajectory_plan_test test/trajectory_plan_test.cpp)

  # Test-Abhängigkeiten setzen
  target_link_libraries(trajectory_plan_test
    psaf_trajectory_plan
  )

  ament_target_dependencies(trajectory_plan_test
  rclcpp
  psaf_configuration
  psaf_interfaces
  OpenCV
  std_msgs
  )

  # Test als ausführbaren Test definieren
  add_test(NAME TrajectoryPlanTest COMMAND trajectory_plan_test)

endif()

add_executable(trajectory_plan src/trajectory_plan.cpp src/trajectory_plan_node.cpp)
ament_target_dependencies(trajectory_plan rclcpp psaf_configuration psaf_interfaces OpenCV std_msgs)                 #.......  according to dependencies
install(TARGETS
        trajectory_plan
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()