cmake_minimum_required(VERSION 3.8)
project(road_sign_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Finde Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Python-Skripte installieren
install(PROGRAMS
  road_sign_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Unit Tests aktivieren, falls BUILD_TESTING gesetzt ist
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  
  ament_add_pytest_tests(road_sign_detector_test
    test/road_sign_detector_test.py
  )
endif()

ament_package()
