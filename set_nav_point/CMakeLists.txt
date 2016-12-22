cmake_minimum_required(VERSION 2.8.3)
project(set_nav_point)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(set_nav_point src/set_nav_point.cpp)
target_link_libraries(set_nav_point
  ${catkin_LIBRARIES}
)
