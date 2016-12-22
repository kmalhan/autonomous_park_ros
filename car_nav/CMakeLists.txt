cmake_minimum_required(VERSION 2.8.3)
project(car_nav)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  navfn
  roscpp
  tf
  teb_local_planner
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/suitesparse
)

add_executable(car_nav_node src/car_nav_node.cpp)
target_link_libraries(car_nav_node
  ${catkin_LIBRARIES}
)
