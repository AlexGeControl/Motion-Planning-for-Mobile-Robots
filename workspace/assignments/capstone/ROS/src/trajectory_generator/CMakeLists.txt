cmake_minimum_required(VERSION 3.14)

project(trajectory_generator)

set(CMAKE_CXX_FLAGS    " ${CMAKE_CXX_FLAGS_INIT} -fPIC")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
)
catkin_package()
include(FetchContent)

# ABSL:
if (NOT TARGET absl::strings OR NOT TARGET absl::status OR NOT TARGET absl::span)
  message(STATUS "osqp-cpp: `absl` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    abseil-cpp
    GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
    GIT_TAG        origin/master
  )
  FetchContent_MakeAvailable(abseil-cpp)
else()
  message(STATUS "osqp-cpp: `absl` targets found.")
endif()

# Eigen3:
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# PCL:
find_package(PCL REQUIRED)

# OSQP:
find_package (osqp REQUIRED)

include_directories(
    include
    SYSTEM 
    third_party
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
)

add_library(osqp-cpp src/osqp++.cpp)
target_link_libraries(
  osqp-cpp 
  PUBLIC absl::strings absl::status Eigen3::Eigen 
  PRIVATE osqp::osqpstatic ${CMAKE_DL_LIBS}
)

add_executable(trajectory_generator_node
	src/trajectory_generator_node.cpp
	src/trajectory_optimizer.cpp
  src/path_finder.cpp
)

target_link_libraries(
  trajectory_generator_node
  
  osqp-cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
