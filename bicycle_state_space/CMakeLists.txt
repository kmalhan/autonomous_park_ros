cmake_minimum_required(VERSION 2.8.3)
project(bicycle_state_space)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
  ugv_course_libs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bicycle_state_space src/bicycle_state_space.cpp)
target_link_libraries(bicycle_state_space
  ${catkin_LIBRARIES}
)

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node
  ${catkin_LIBRARIES}
)
