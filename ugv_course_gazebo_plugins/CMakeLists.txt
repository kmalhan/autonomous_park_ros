cmake_minimum_required(VERSION 2.8.3)
project(ugv_course_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  ugv_course_libs
  visualization_msgs
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/GpsPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_plugin pose_plugin
  DEPENDS gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

add_library(gps_plugin src/GpsPlugin.cpp)
add_dependencies(gps_plugin ${PROJECT_NAME}_gencfg)
target_link_libraries(gps_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(pose_plugin src/PosePlugin.cpp)
add_dependencies(pose_plugin ${PROJECT_NAME}_gencfg)
target_link_libraries(pose_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(view_control_plugin src/ViewControlPlugin.cpp)
add_dependencies(view_control_plugin ${PROJECT_NAME}_gencfg)
target_link_libraries(view_control_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
)
