cmake_minimum_required (VERSION 2.8)
project (resource_management_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    )

add_message_files(
    FILES
    StateMachineHeader.msg
    StateMachineTransition.msg
    EndCondition.msg
    MessagePriority.msg
    StateMachineStateHeader.msg
    StateMachinesStatus.msg
    PrioritiesSetter.msg
)

add_service_files(
    FILES
    StateMachinesCancel.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package( CATKIN_DEPENDS message_runtime std_msgs)
