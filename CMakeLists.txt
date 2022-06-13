cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 Boost
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/Roadmap/roadmap.cpp
   src/Utility/utility.cpp
   src/Dubins/dubins.cpp
   src/Log/logger.cpp
   src/World_representation/world_representation.cpp
   src/Robot_manager/robot_manager.cpp
   src/Robot_manager/robots.cpp
   src/student_interface.cpp
   src/Connector/connector.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}  
)
