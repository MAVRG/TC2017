cmake_minimum_required(VERSION 2.8.3)
project(cnnjoyteleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  sensor_msgs
  std_msgs
  roscpp
)

catkin_package(
  LIBRARIES cnnjoyteleop
  CATKIN_DEPENDS geometry_msgs joy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(cnnjoyteleop src/cnnjoyteleop.cpp)

target_link_libraries(cnnjoyteleop ${catkin_LIBRARIES})
