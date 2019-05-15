#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H

#include <string>
#include <map>
#include <mutex>

#include "resource_synchronizer/StateMachine.h"
#include "resource_management/tools/StateMachineServer.h"

namespace resource_synchronizer
{

struct StateMachinePriority
{
  int priority;
  int state_machine_id;
};

template<typename SMT, typename RMT>
class StateMachinesHolder
{
  typedef StateMachine<typename SMT::_state_machine_type> state_machine_type_;

public:
  StateMachinesHolder(std::string name) : server_(name, true)
  {
    server_.waitForServer();
    server_.registerSatusCallback([this](auto status){ this->stateMachineStatus(status); });
    running_id_ = -1;
  }

  void insert(int id, SMT sub_state_machine, resource_management_msgs::MessagePriority importance)
  {
    state_machines_.insert( std::pair<int, state_machine_type_>
      (id, state_machine_type_(sub_state_machine.state_machine, sub_state_machine.header, importance) ) );
  }

  bool send(int id)
  {
    if(running_id_ != -1)
      return false;

    auto it = state_machines_.find(id);
    if(it != state_machines_.end())
    {
      RMT registerMsg;
      registerMsg.request.state_machine = it->second.getStateMachineMsg();
      registerMsg.request.header = it->second.getHeaderMsg();
      server_.send(registerMsg);
      running_id_ = id;
      return true;
    }
    else
      return false;
  }

  bool cancel()
  {
    if(running_id_ != -1)
    {
      server_.cancel();
      server_.waitForResult();
      return true;
    }
    return false;
  }

  int isRunning()
  {
    return running_id_;
  }

  StateMachinePriority getHighestPriority()
  {
    mutex_.lock();
    StateMachinePriority highest;
    highest.priority = -100;
    highest.state_machine_id = -1;

    for(const auto& it : state_machines_)
    {
      if(it.second.getPriority() > highest.priority)
      {
        highest.priority = it.second.getPriority();
        highest.state_machine_id = it.first;
      }
    }
    mutex_.unlock();

    return highest;
  }

private:
  std::map<int, state_machine_type_> state_machines_;
  int running_id_;
  std::mutex mutex_;

  resource_management::StateMachineServer<RMT> server_;

  void stateMachineStatus(resource_management::stateMachineState_t status)
  {
    if(running_id_ == -1)
    {
      if(status.state_name_ == "")
      {
        mutex_.lock();
        running_id_ = -1;
        auto it = state_machines_.find(running_id_);
        state_machines_.erase(it);
        mutex_.unlock();
      }
      std::cout << status.toString() << std::endl;
    }
  }
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H