cmake_minimum_required (VERSION 3.5)
project (resource_management)

add_compile_options(-std=c++14 -Wall)

include_directories(
  include
)

add_library(state_machine_lib
  src/state_machine/CoordinationTransition.cpp
  src/state_machine/CoordinationState.cpp
  src/state_machine/CoordinationStateMachine.cpp
)

add_executable(state_machine src/st_main.cpp)
target_link_libraries(state_machine state_machine_lib)
