cmake_minimum_required(VERSION 3.8)
project(goals)

# Opciones de compilación recomendadas
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias de ROS2 necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Ejecutable
add_executable(version2_publisher src/publisher.cpp)

# Vinculación de dependencias
ament_target_dependencies(version2_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  nav2_msgs
)

# Instalación
install(TARGETS
  version2_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Tests opcionales
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

