cmake_minimum_required(VERSION 3.0.2)
project(maddrive_teleop)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMD_VEL_FILTER_NAME cmd_vel_filter)
set(CMD_VEL_FILTER_SRC src/cmd_filter/main.cpp)

add_executable(
  ${CMD_VEL_FILTER_NAME} ${CMD_VEL_FILTER_SRC}
)

target_link_libraries(
  ${CMD_VEL_FILTER_NAME}
  ${catkin_LIBRARIES}
)
