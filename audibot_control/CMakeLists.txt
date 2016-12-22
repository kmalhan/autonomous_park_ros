cmake_minimum_required(VERSION 2.8.3)
project(audibot_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES audi_speed_controller audi_steering_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(actuator_sim src/ActuatorSim.cpp)
target_link_libraries(actuator_sim
  ${catkin_LIBRARIES}
)

add_library(audi_speed_controller src/AudiSpeedControl.cpp)
target_link_libraries(audi_speed_controller
  ${catkin_LIBRARIES}
  actuator_sim
)

add_library(audi_steering_controller src/AudiSteeringControl.cpp)
target_link_libraries(audi_steering_controller
  ${catkin_LIBRARIES}
)

add_executable(audi_twist_controller src/twist_controller.cpp)
target_link_libraries(audi_twist_controller
  ${catkin_LIBRARIES}
)

install(TARGETS audi_speed_controller audi_steering_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES audibot_control_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
