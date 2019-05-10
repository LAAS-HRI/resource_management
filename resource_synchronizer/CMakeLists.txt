cmake_minimum_required (VERSION 2.8)
project (resource_synchronizer)

add_compile_options(-std=c++14 -Wall -Wextra -pthread)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    resource_management_msgs
    resource_synchronizer_msgs
    )

catkin_package(
#    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp resource_management_msgs resource_synchronizer_msgs
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)
