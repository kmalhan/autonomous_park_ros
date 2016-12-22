cmake_minimum_required(VERSION 2.8.3)
project(img_transform)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  image_view
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(img_transform_node src/img_transform_node.cpp)
target_link_libraries(img_transform_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

