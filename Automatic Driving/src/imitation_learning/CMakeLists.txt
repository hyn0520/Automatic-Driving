cmake_minimum_required(VERSION 3.8)
project(imitation_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Finde ROS 2-Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Installiere die Python-Skripte
install(
  DIRECTORY imitation_learning/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Installiere das Modell
install(
  FILES behavior_clone_model.pth
  DESTINATION share/${PROJECT_NAME}
)

# Python-Abhängigkeiten registrieren
ament_python_install_package(${PROJECT_NAME})

# Unit Tests aktivieren, falls BUILD_TESTING gesetzt ist
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  
  ament_add_pytest_tests(
    test_imitation_learning
    test/imitation_learning_test.py  # <-- Korrekte Referenz auf die Datei
  )
endif()

ament_package()
