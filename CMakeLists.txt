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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(faces_interfaces REQUIRED)
find_package(human_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# 2. Librería
add_library(${PROJECT_NAME} src/Vision.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  vision_interfaces
  faces_interfaces
  sensor_msgs
  std_srvs
  human_interfaces
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 3. Nodo Executable
add_executable(detect_seats_node src/detect_seats_node.cpp)
add_executable(Detect_Fusion_Node src/Detect_Fusion_Node.cpp)
add_executable(image_fixed_node src/image_fixed_node.cpp)

ament_target_dependencies(detect_seats_node
  rclcpp
  std_srvs
  sensor_msgs
  vision_interfaces
  tf2_ros
  geometry_msgs
  OpenCV
  cv_bridge
)

target_link_libraries(detect_seats_node ${OpenCV_LIBS})

ament_target_dependencies(Detect_Fusion_Node
  rclcpp
  std_srvs
  sensor_msgs
  vision_interfaces
  tf2_ros
  geometry_msgs
)

ament_target_dependencies(image_fixed_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_fixed_node ${OpenCV_LIBS})

# 4. Instalación
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS detect_seats_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS Detect_Fusion_Node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Agregamos la instalación de tu nuevo nodo
install(TARGETS image_fixed_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# 5. Exportación
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  vision_interfaces
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  faces_interfaces
  human_interfaces
  cv_bridge
  OpenCV
)

ament_package()
