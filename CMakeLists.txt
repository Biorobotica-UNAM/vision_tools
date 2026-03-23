cmake_minimum_required(VERSION 3.8)
project(vision_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(faces_interfaces REQUIRED)

# 2. Librería
add_library(${PROJECT_NAME} src/Vision.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  vision_interfaces
)

# 3. Nodo Executable
add_executable(detect_seats_node src/detect_seats_node.cpp)

ament_target_dependencies(detect_seats_node
  rclcpp
  std_srvs
  sensor_msgs
  vision_interfaces
  tf2_ros
  geometry_msgs
)

# 4. Instalación (Aquí está la solución al error)
install(TARGETS
  ${PROJECT_NAME}
  detect_seats_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# 5. Exportación
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  vision_interfaces
  sensor_msgs
  tf2_ros
  std_srvs
)

ament_package()