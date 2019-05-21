cmake_minimum_required (VERSION 2.8)
project (resource_synchronizer)

add_compile_options(-std=c++14 -Wall -Wextra -pthread)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    resource_management_msgs
    resource_management
    resource_synchronizer_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES resource_synchronizer_lib
    CATKIN_DEPENDS roscpp resource_management_msgs resource_synchronizer_msgs
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)

add_library(synchronizer_lib
  src/synchronizer/StateMachineSynchroHolder.cpp
)
add_dependencies(synchronizer_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(resource_synchronizer_lib
  src/StateMachinesManager.cpp
)
add_dependencies(resource_synchronizer_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
