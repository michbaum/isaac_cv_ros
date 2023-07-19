cmake_minimum_required(VERSION 2.8.3)
project(isaac_cv_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# TODO: (michbaum) Might need to add dependencies here
