cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
)
add_library(astar src/dyn_a_star.cpp)
add_executable(global_planning src/global_planning.cpp src/dyn_a_star.cpp)

target_link_libraries(global_planning
  astar
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
