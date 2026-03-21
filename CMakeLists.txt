cmake_minimum_required(VERSION 3.8)
project(vision_tools)

find_package(ament_cmake REQUIRED)

# Instalar carpeta launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()