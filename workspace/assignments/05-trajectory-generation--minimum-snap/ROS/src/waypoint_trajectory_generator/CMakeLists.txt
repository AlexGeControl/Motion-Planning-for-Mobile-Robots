cmake_minimum_required(VERSION 3.14)

project(waypoint_trajectory_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  quadrotor_msgs
)
catkin_package()

include(cmake/eigen3.cmake)
include(cmake/absl.cmake)
include(cmake/osqp.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(osqp-cpp STATIC src/osqp++.cpp)
target_link_libraries(osqp-cpp PUBLIC absl::strings absl::status Eigen3::Eigen PRIVATE osqp::osqpstatic ${CMAKE_DL_LIBS})
target_include_directories(osqp-cpp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(
  trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)

target_link_libraries(
  trajectory_generator_node
  
  PUBLIC
  osqp-cpp

  ${catkin_LIBRARIES}
)
