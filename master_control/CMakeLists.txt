cmake_minimum_required(VERSION 2.8.3)
project(master_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(master_node src/master_node.cpp)
target_link_libraries(master_node
  ${catkin_LIBRARIES}
)

add_executable(parking_movement src/parking_movement.cpp)
target_link_libraries(parking_movement
  ${catkin_LIBRARIES}
)
