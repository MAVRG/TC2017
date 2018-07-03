cmake_minimum_required(VERSION 2.8.3)
project(joyteleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  sensor_msgs
  std_msgs
  roscpp
)

catkin_package(
  LIBRARIES joyteleop
  CATKIN_DEPENDS geometry_msgs joy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(joyteleop src/joyteleop.cpp)

target_link_libraries(joyteleop ${catkin_LIBRARIES})


