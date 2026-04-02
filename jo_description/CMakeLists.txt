cmake_minimum_required(VERSION 3.5.1)
project(jo_description)

find_package(ament_cmake)


install(
  DIRECTORY config launch urdf worlds rviz missions
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(scripts)

install(
  PROGRAMS  scripts/send_gps_waypoints.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()