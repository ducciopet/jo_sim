cmake_minimum_required(VERSION 3.5.1)
project(jo_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(
  DIRECTORY config launch worlds rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/turtlebot_obstacle_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
