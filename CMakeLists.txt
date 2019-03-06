cmake_minimum_required (VERSION 3.5)
project (resource_management)

add_compile_options(-std=c++14 -Wall -pthread)

find_package (Threads)

include_directories(
  include
)

add_library(state_machine_lib
  src/state_machine/CoordinationTransition.cpp
  src/state_machine/CoordinationState.cpp
  src/state_machine/CoordinationStateMachine.cpp
  src/state_machine/EventStorage.cpp
  src/state_machine/StateStorage.cpp
  src/state_machine/CoordinationSignalsStorage.cpp
)

add_library(message_lib
  src/message_storage/ReactiveBuffer.cpp
  src/message_storage/ReactiveBufferStorage.cpp
)

add_executable(state_machine src/st_main.cpp)
target_link_libraries(state_machine state_machine_lib)
target_link_libraries(state_machine ${CMAKE_THREAD_LIBS_INIT})

add_executable(signals src/sig_main.cpp)
target_link_libraries(signals message_lib)
target_link_libraries(signals ${CMAKE_THREAD_LIBS_INIT})
