#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H

#include <vector>

#include "resource_synchronizer/StateMachine.h"
#include "resource_management/tools/StateMachineServer.h"

namespace resource_synchronizer
{
// SMT = led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs
//-->SMT = led_manager_msgs::StateMachine
//RMT = led_manager_msgs::StateMachineRegister
template<typename SMT, typename RMT>
class StateMachinesHolder
{
public:
  StateMachinesHolder(std::string name) : server_(name) {}

  void insert(SMT sub_state_machine, resource_management_msgs::MessagePriority importance)
  {
    state_machines_.push_back(StateMachine<typename SMT::_state_machine_type>(sub_state_machine.state_machine, sub_state_machine.header, importance));
  }

private:
  std::vector<StateMachine<typename SMT::_state_machine_type>> state_machines_;

  resource_management::StateMachineServer<RMT> server_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
