#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINE_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINE_H

#include <string>

#include <ros/ros.h>

#include "resource_synchronizer_msgs/SubStateMachineHeader.h"
#include "resource_management_msgs/MessagePriority.h"
#include "resource_management_msgs/StateMachineHeader.h"

namespace resource_synchronizer
{

template<typename T>
class StateMachine
{
public:
  StateMachine(T state_machine_msg, resource_synchronizer_msgs::SubStateMachineHeader header, resource_management_msgs::MessagePriority importance)
  {
    state_machine_ = state_machine_msg;
    header_.initial_state = header.initial_state;
    header_.timeout = header.timeout;
    header_.begin_dead_line = header.begin_dead_line;
    header_.priority = importance;
  }

  bool isTooLate()
  {
    if(ros::Time(0) > header_.begin_dead_line)
      return true;
    else
      return false;
  }

  resource_management_msgs::StateMachineHeader getHeaderMsg()
  {
    return header_;
  }

  T getStateMachineMsg()
  {
    return state_machine_;
  }

  T operator()()
  {
    return state_machine_;
  }

  int getPriority()
  {
    return header_.priority;
  }

private:
  T state_machine_;
  resource_management_msgs::StateMachineHeader header_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINE_H
