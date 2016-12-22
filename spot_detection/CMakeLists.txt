cmake_minimum_required(VERSION 2.8.3)
project(spot_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/lane_extract.cfg
  cfg/vehicle_control.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_extract src/lane_extract.cpp)
add_dependencies(lane_extract lane_extract_gencfg)
target_link_libraries(lane_extract
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(feature2d src/feature2d.cpp)
add_dependencies(lane_extract lane_extract_gencfg)
target_link_libraries(feature2d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(vehicle_control src/vehicle_control.cpp)
add_dependencies(vehicle_control vehicle_control_gencfg)
target_link_libraries(vehicle_control
  ${catkin_LIBRARIES}
)

add_executable(find_parking src/find_parking.cpp)
target_link_libraries(find_parking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(image_transform src/image_transform.cpp)
target_link_libraries(image_transform
  ${catkin_LIBRARIES}
)

