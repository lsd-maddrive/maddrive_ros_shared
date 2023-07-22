cmake_minimum_required(VERSION 3.0.2)
project(maddrive_worlds)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
