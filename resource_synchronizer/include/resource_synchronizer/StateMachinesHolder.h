#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H

#include <string>
#include <map>
#include <mutex>
#include <functional>

#include "resource_synchronizer/StateMachine.h"
#include "resource_management/tools/StateMachineServer.h"

#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"

namespace resource_synchronizer
{

struct StateMachinePriority
{
  int priority;
  int state_machine_id;
};

class StateMachinesHolderBase
{
public:
  virtual bool send(int id) = 0;
  virtual bool cancel() = 0;

  virtual int isRunning() = 0;
  virtual bool isOneRunning(int id) = 0;

  virtual std::vector<StateMachinePriority> getPriorities() = 0;
  virtual void clean() = 0;
};

template<typename SMT, typename RMT>
class StateMachinesHolder : public StateMachinesHolderBase
{
  typedef StateMachine<typename SMT::_state_machine_type> state_machine_type_;

public:
  StateMachinesHolder(std::string name) : server_(name, true)
  {
    server_.waitForServer();
    server_.registerSatusCallback([this](auto status){ this->stateMachineStatus(status); });
    running_id_ = -1;
    name_ = name;
  }

  void insert(int id, SMT sub_state_machine, resource_management_msgs::MessagePriority importance)
  {
    if(sub_state_machine.header.initial_state != "")
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

  bool isOneRunning(int id)
  {
    return running_id_ == id;
  }

  std::vector<StateMachinePriority> getPriorities()
  {
    std::vector<StateMachinePriority> priorities;

    mutex_.lock();

    for(auto it : state_machines_)
    {
      StateMachinePriority priority;
      priority.priority = it.second.getPriority();
      priority.state_machine_id = it.first;

      size_t index = 0;
      for(size_t i = 0; i < priorities.size(); i++)
      {
        if(priority.priority > priorities[i].priority)
        {
          index = i;
          break;
        }
      }
      priorities.insert(priorities.begin() + index, priority);
    }
    mutex_.unlock();

    return priorities;
  }

  void clean()
  {
    for(auto it = state_machines_.begin(); it != state_machines_.end();)
    {
      if(it->second.isTooLate())
        state_machines_.erase(it);
      else
        ++it;
    }
  }

  void registerSatusCallback(std::function<void(resource_synchronizer_msgs::MetaStateMachinesStatus)> status_callback) { status_callback_ = status_callback; }

private:
  std::map<int, state_machine_type_> state_machines_;
  int running_id_;
  std::mutex mutex_;
  std::string name_;

  resource_management::StateMachineServer<RMT> server_;
  std::function<void(resource_synchronizer_msgs::MetaStateMachinesStatus)> status_callback_;

  void stateMachineStatus(resource_management::stateMachineState_t status)
  {
    if(running_id_ == -1)
    {
      resource_synchronizer_msgs::MetaStateMachinesStatus msg;
      msg.id = running_id_;
      msg.resource = name_;
      msg.state_name = status.state_name_;
      msg.state_event = status.state_event_;
      if(status_callback_)
        status_callback_(msg);

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
