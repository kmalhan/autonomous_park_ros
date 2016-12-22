
cmake_minimum_required(VERSION 2.8.3)
project(ugv_course_libs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
)

