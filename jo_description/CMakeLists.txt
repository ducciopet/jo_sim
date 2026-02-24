cmake_minimum_required(VERSION 3.5.1)
project(jo_description)

find_package(ament_cmake)


install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()