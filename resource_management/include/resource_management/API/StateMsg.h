#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMSG_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMSG_H_

#include <string>
#include <vector>

#include <ros/ros.h>

#include "resource_management_msgs/StateMachineTransition.h"

namespace resource_management
{

template<typename ST>
class StateMsg
{
  typedef typename ST::_data_type DataType;
public:
  StateMsg(const std::string& name, DataType data)
  {
    state_.header.id = name;
    state_.data = data;
  }

  void addTransition(const std::string& to, ros::Duration timeout, ros::Duration duration, const std::vector<std::string>& regexs = {})
  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = to;
    transition.end_condition.timeout = timeout;
    transition.end_condition.duration = duration;
    transition.end_condition.regex_end_condition = regexs;
    state_.header.transitions.push_back(transition);
  }

  ST operator()() { return state_; }
private:
  ST state_;
};

} // resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMSG_H_
