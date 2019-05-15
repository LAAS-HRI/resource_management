cmake_minimum_required (VERSION 2.8)
project (resource_synchronizer_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    resource_management_msgs
    )

add_message_files(
    FILES
    MetaStateMachinesStatus.msg
    MetaStateMachineHeader.msg
    SubStateMachineHeader.msg
)
generate_messages(DEPENDENCIES std_msgs resource_management_msgs)

catkin_package( CATKIN_DEPENDS message_runtime std_msgs resource_management_msgs)
