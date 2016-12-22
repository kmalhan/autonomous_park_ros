cmake_minimum_required(VERSION 2.8.3)
project(ugv_course_gazebo)

find_package(catkin REQUIRED)

catkin_package()

FILE(GLOB_RECURSE files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*.tar.gz")
FOREACH(file_ ${files})
  MESSAGE(STATUS "Extracting file: ${file_}")
  get_filename_component(file_path ${file_} DIRECTORY)
  get_filename_component(file_name ${file_} NAME)
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${file_name}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${file_path}
  )
ENDFOREACH(file_)
