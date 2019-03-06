cmake_minimum_required (VERSION 2.8)
project (resource_management)

add_compile_options(-std=c++14 -Wall -pthread)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    )

add_message_files(
    FILES
    CoordinationStateHeader.msg
    CoordinationTransition.msg
    EndCondition.msg
    PriorityTargetHeader.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES message_lib state_machine_lib
    CATKIN_DEPENDS roscpp message_runtime std_msgs
    # DEPENDS
    )

find_package (Threads)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)

add_library(state_machine_lib
  src/state_machine/CoordinationTransition.cpp
  src/state_machine/CoordinationState.cpp
  src/state_machine/CoordinationStateMachine.cpp
  src/state_machine/EventStorage.cpp
  src/state_machine/StateStorage.cpp
  src/state_machine/CoordinationSignalsStorage.cpp
)
add_dependencies(state_machine_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(message_lib
  src/message_storage/ReactiveBuffer.cpp
  src/message_storage/ReactiveBufferStorage.cpp
)

add_executable(state_machine src/st_main.cpp)
add_dependencies(state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES} state_machine_lib)
target_link_libraries(state_machine ${CMAKE_THREAD_LIBS_INIT})

add_executable(signals src/sig_main.cpp)
target_link_libraries(signals message_lib)
target_link_libraries(signals ${CMAKE_THREAD_LIBS_INIT})

add_executable(ros_demo src/ros_main.cpp)

add_dependencies(ros_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_demo ${catkin_LIBRARIES} message_lib state_machine_lib)
