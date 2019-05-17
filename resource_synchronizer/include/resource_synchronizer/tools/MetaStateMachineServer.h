#ifndef _RESOURCE_SYNCHRONIZER_INCLUDE_RESOURCE_SYNCHRONIZER_METASTATEMACHINESERVER_H_
#define _RESOURCE_SYNCHRONIZER_INCLUDE_RESOURCE_SYNCHRONIZER_METASTATEMACHINESERVER_H_

#include <functional>
#include <thread>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"
#include "resource_management_msgs/StateMachinesCancel.h"

namespace resource_management
{

struct MetaStateMachineState_t
{
  MetaStateMachineState_t() {}

  std::string toString()
  {
    std::string res;
    for(size_t i = 0; i < sub_state_machine_name_.size(); i++)
      res += "[" + sub_state_machine_name_[i] + "]" + state_name_[i] + " : " + state_event_[i] + "\n";
    return res;
  }

  bool end()
  {
    bool res = true;
    if(state_name_.size() == 0)
      res = false;
    else
    {
      for(auto name : state_name_)
        res = res && (name == "");
    }

    return res;
  }

  void clear()
  {
    sub_state_machine_name_.clear();
    state_name_.clear();
    state_event_.clear();
  }

  void set(const std::vector<std::string>& sub_state_machine_name, const std::vector<std::string>& state_name, const std::vector<std::string>& state_event)
  {
    sub_state_machine_name_ = sub_state_machine_name;
    state_name_ = state_name;
    state_event_ = state_event;
  }

  std::vector<std::string> sub_state_machine_name_;
  std::vector<std::string> state_name_;
  std::vector<std::string> state_event_;
};

template<class MessageType>
class MetaStateMachineServer
{
public:
  MetaStateMachineServer(std::string name, bool spin_thread = true);
  ~MetaStateMachineServer();

  void waitForServer(ros::Duration timeout = ros::Duration(-1));
  void waitForServer(int32_t timeout);

  bool send(MessageType srv);
  bool waitForResult(ros::Duration timeout = ros::Duration(-1));
  MetaStateMachineState_t getResult() { return state_; }
  bool cancel();

  void registerSatusCallback(std::function<void(MetaStateMachineState_t)> status_callback) { status_callback_ = status_callback; }

private:
  ros::NodeHandle nh_;
  std::string name_;
  int id_;

  std::mutex terminate_mutex_;
  bool need_to_terminate_;
  std::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;

  std::string register_topic_name_;
  std::string cancel_topic_name_;
  std::string status_topic_name_;
  ros::Subscriber status_subscriber_;

  std::function<void(MetaStateMachineState_t)> status_callback_;

  MetaStateMachineState_t state_;

  void init(ros::NodeHandle& n, bool spin_thread);
  void spinThread();
  void statusCallback(const resource_synchronizer_msgs::MetaStateMachinesStatus msg);
};

template<class MessageType>
MetaStateMachineServer<MessageType>::MetaStateMachineServer(std::string name, bool spin_thread)
{
  name_ = "/" + name;
  id_ = -1;

  init(nh_, spin_thread);

  register_topic_name_ = name_ + "/state_machines_register";
  cancel_topic_name_ = name_ + "/state_machine_cancel";
  status_topic_name_ = name_ + "/state_machine_status";

  status_subscriber_ = nh_.subscribe(status_topic_name_, 100, &MetaStateMachineServer::statusCallback, this);
}

template<class MessageType>
MetaStateMachineServer<MessageType>::~MetaStateMachineServer()
{
  if(spin_thread_)
  {
    terminate_mutex_.lock();
    need_to_terminate_ = true;
    terminate_mutex_.unlock();
    spin_thread_->join();
    delete spin_thread_;
  }
}

template<class MessageType>
void MetaStateMachineServer<MessageType>::waitForServer(ros::Duration timeout)
{
  ros::service::waitForService(register_topic_name_, timeout);
}

template<class MessageType>
void MetaStateMachineServer<MessageType>::waitForServer(int32_t timeout)
{
  ros::service::waitForService(register_topic_name_, timeout);
}

template<class MessageType>
bool MetaStateMachineServer<MessageType>::send(MessageType srv)
{
  ros::ServiceClient client = nh_.serviceClient<MessageType>(register_topic_name_);

  if(client.call(srv))
  {
    id_ = srv.response.id;
    state_.clear();
    return true;
  }
  else
    return false;
}

template<class MessageType>
bool MetaStateMachineServer<MessageType>::waitForResult(ros::Duration timeout)
{
  ros::Time strat = ros::Time(0);
  bool end = false;

  while((end == false) && ((timeout == ros::Duration(-1)) || (ros::Time(0) - strat <= timeout)) && ros::ok())
  {
    if(state_.end())
    {
      end = true;
      continue;
    }
  }

  return end;
}

template<class MessageType>
bool MetaStateMachineServer<MessageType>::cancel()
{
  if(id_ == -1)
    return false;

  ros::ServiceClient client = nh_.serviceClient<resource_management_msgs::StateMachinesCancel>(cancel_topic_name_);
  resource_management_msgs::StateMachinesCancel srv;
  srv.request.id = id_;

  if(client.call(srv))
    return srv.response.ack;
  else
    return false;
}

template<class MessageType>
void MetaStateMachineServer<MessageType>::init(ros::NodeHandle& n, bool spin_thread)
{
  if(spin_thread)
  {
    ROS_DEBUG("Spinning up a thread for the MetaStateMachineServer");
    need_to_terminate_ = false;
    n.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&MetaStateMachineServer<MessageType>::spinThread, this));
  }
  else
  {
    spin_thread_ = nullptr;
  }
}

template<class MessageType>
void MetaStateMachineServer<MessageType>::spinThread()
{
  while(nh_.ok())
  {
    terminate_mutex_.lock();
    if(need_to_terminate_)
      break;
    terminate_mutex_.unlock();
    callback_queue_.callAvailable(ros::WallDuration(0.1));
  }
}

template<class MessageType>
void MetaStateMachineServer<MessageType>::statusCallback(const resource_synchronizer_msgs::MetaStateMachinesStatus msg)
{
  if((int)msg.id == id_)
  {
    state_.set(msg.resource, msg.state_name, msg.state_event);

    if(status_callback_)
      status_callback_(state_);
  }
}

} // namespace resource_management

#endif // _RESOURCE_SYNCHRONIZER_INCLUDE_RESOURCE_SYNCHRONIZER_METASTATEMACHINESERVER_H_
