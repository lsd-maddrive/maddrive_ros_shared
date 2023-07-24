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

set(STEERING_FILTER_NAME steering_filter)
set(STEERING_FILTER_SRC src/cmd_filter/main.cpp)

add_executable(
  ${STEERING_FILTER_NAME} ${STEERING_FILTER_SRC}
)

target_link_libraries(
  ${STEERING_FILTER_NAME}
  ${catkin_LIBRARIES}
)
