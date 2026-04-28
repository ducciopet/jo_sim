cmake_minimum_required(VERSION 3.5.1)
project(jo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/turtlebot_obstacle_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
