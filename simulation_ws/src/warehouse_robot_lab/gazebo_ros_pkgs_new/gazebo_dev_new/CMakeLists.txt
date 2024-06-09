cmake_minimum_required(VERSION 2.8.3)
project(gazebo_dev_new)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS gazebo_dev-extras.cmake
)
