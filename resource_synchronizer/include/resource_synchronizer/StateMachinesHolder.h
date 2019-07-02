#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H

#include <string>
#include <map>
#include <mutex>
#include <functional>

#include <ros/ros.h>

#include "resource_synchronizer/StateMachine.h"
#include "resource_management/API/StateMachineClient.h"

namespace resource_synchronizer
{

struct StateMachinePriority
{
  int priority;
  int state_machine_id;
};


struct SubStateMachineStatus
{
  std::string resource;
  std::string state_name;
  std::string event_name;
  int id;
};

class StateMachinesHolderBase
{
public:
  virtual bool send(int id) = 0;
  virtual bool cancel() = 0;
  virtual bool cancel(int id) = 0;

  virtual int isRunning() = 0;
  virtual bool isOneRunning(int id) = 0;
  virtual bool canReplace(int id) = 0;

  virtual std::vector<int> getIdsPerPriorities() = 0;
  virtual void clean() = 0;
};

template<typename SMT, typename RMT, typename SMET>
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

  bool insert(int id, SMT sub_state_machine, resource_management_msgs::MessagePriority importance)
  {
    if(sub_state_machine.header.initial_state != "")
    {
      state_machines_.insert( std::pair<int, state_machine_type_>
        (id, state_machine_type_(sub_state_machine.state_machine, sub_state_machine.header, importance) ) );
      return true;
    }
    else
      return false;
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

  bool cancel(int id)
  {
    if(running_id_ == id)
    {
      server_.cancel();
      server_.waitForResult();
    }
    else
      state_machines_.erase(id);

    return true;
  }

  int isRunning()
  {
    return running_id_;
  }

  bool isOneRunning(int id)
  {
    return running_id_ == id;
  }

  std::vector<int> getIdsPerPriorities()
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

    std::vector<int> res;
    for(const auto& x : priorities)
      res.push_back(x.state_machine_id);

    return res;
  }

  bool canReplace(int id)
  {
    if(running_id_ == -1)
      return true;
    else
    {
      auto it_id = state_machines_.find(id);
      auto it_running_id = state_machines_.find(running_id_);
      if(it_id->second.getHeaderMsg().priority.value > it_running_id->second.getHeaderMsg().priority.value)
        return true;
      else
        return false;
    }
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

  void registerSatusCallback(std::function<void(SubStateMachineStatus)> status_callback) { status_callback_ = status_callback; }

  std::vector<std::string> getSynchros(int id)
  {
    std::vector<std::string> res;

    auto it = state_machines_.find(id);
    if(it != state_machines_.end())
    {
      SMET srv;
      srv.request.state_machine = it->second.getStateMachineMsg();
      ros::NodeHandle nh;
      ros::ServiceClient client = nh.serviceClient<SMET>(name_ + "/extract_synchro__");
      if (client.call(srv))
        res = srv.response.synchros;
    }

    return res;
  }

private:
  std::map<int, state_machine_type_> state_machines_;
  int running_id_;
  std::mutex mutex_;
  std::string name_;

  resource_management::StateMachineClient<RMT> server_;
  std::function<void(SubStateMachineStatus)> status_callback_;

  void stateMachineStatus(resource_management::stateMachineState_t status)
  {
    if(running_id_ != -1)
    {
      SubStateMachineStatus sub_status;
      sub_status.id = running_id_;
      sub_status.resource = name_;
      sub_status.state_name = status.state_name_;
      sub_status.event_name = status.state_event_;
      if(status_callback_)
        status_callback_(sub_status);

      if(status.state_name_ == "")
      {
        mutex_.lock();
        state_machines_.erase(running_id_);
        running_id_ = -1;
        mutex_.unlock();
      }
    }
  }
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESHOLDER_H
