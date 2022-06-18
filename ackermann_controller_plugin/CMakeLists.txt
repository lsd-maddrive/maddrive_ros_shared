cmake_minimum_required(VERSION 3.0.2)
project(ackermann_controller_plugin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    gazebo_plugins
)
find_package(gazebo REQUIRED)

set(CAR_INTERFACE_PLUGIN ackermann_controller_plugin)

include_directories(
    include
    ${Boost_INCLUDE_DIR} 
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
  gazebo_plugins
  LIBRARIES
  CATKIN_DEPENDS
)

add_library(${CAR_INTERFACE_PLUGIN}
    src/plugin.cpp
)

target_link_libraries(${CAR_INTERFACE_PLUGIN}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)
