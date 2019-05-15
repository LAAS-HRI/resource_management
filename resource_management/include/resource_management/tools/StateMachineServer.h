#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMACHINESERVER_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMACHINESERVER_H_

#include <functional>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "resource_management_msgs/StateMachinesStatus.h"
#include "resource_management_msgs/StateMachinesCancel.h"

namespace resource_management
{

struct stateMachineState_t
{
  stateMachineState_t() {}
  stateMachineState_t(const std::string& state_name, const std::string& state_event)
  {
    state_name_ = state_name;
    state_event_ = state_event;
  }

  std::string toString()
  {
    return state_name_ + " : " + state_event_;
  }

  std::string state_name_;
  std::string state_event_;
};

template<class MessageType>
class StateMachineServer
{
public:
  StateMachineServer(std::string name, bool synchronised = false, bool spin_thread = true);
  ~StateMachineServer();

  void waitForServer(ros::Duration timeout = ros::Duration(-1));
  void waitForServer(int32_t timeout);

  bool send(MessageType srv);
  bool waitForResult(ros::Duration timeout = ros::Duration(-1));
  stateMachineState_t getResult() { return state_; }
  bool cancel();

  void registerSatusCallback(std::function<void(stateMachineState_t)> status_callback) { status_callback_ = status_callback; }

private:
  ros::NodeHandle nh_;
  std::string name_;
  int id_;
  bool synchronised_;

  std::mutex terminate_mutex_;
  bool need_to_terminate_;
  std::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;

  std::string register_topic_name_;
  std::string cancel_topic_name_;
  std::string status_topic_name_;
  ros::Subscriber status_subscriber_;

  std::function<void(stateMachineState_t)> status_callback_;

  stateMachineState_t state_;

  void init(ros::NodeHandle& n, bool spin_thread);
  void spinThread();
  void statusCallback(const resource_management_msgs::StateMachinesStatus msg);
};

template<class MessageType>
StateMachineServer<MessageType>::StateMachineServer(std::string name, bool synchronised, bool spin_thread)
{
  name_ = "/" + name;
  id_ = -1;
  synchronised_ = synchronised;

  init(nh_, spin_thread);

  register_topic_name_ = synchronised_ ? name_ + "/state_machines_register__" : name_ + "/state_machines_register";
  cancel_topic_name_ = synchronised_ ? name_ + "/state_machine_cancel__" : name_ + "/state_machine_cancel";
  status_topic_name_ = synchronised_ ? name_ + "/state_machine_status__" : name_ + "/state_machine_status";

  status_subscriber_ = nh_.subscribe(status_topic_name_, 100, &StateMachineServer::statusCallback, this);
}

template<class MessageType>
StateMachineServer<MessageType>::~StateMachineServer()
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
void StateMachineServer<MessageType>::waitForServer(ros::Duration timeout)
{
  ros::service::waitForService(register_topic_name_, timeout);
}

template<class MessageType>
void StateMachineServer<MessageType>::waitForServer(int32_t timeout)
{
  ros::service::waitForService(register_topic_name_, timeout);
}

template<class MessageType>
bool StateMachineServer<MessageType>::send(MessageType srv)
{
  ros::ServiceClient client = nh_.serviceClient<MessageType>(register_topic_name_);

  if(client.call(srv))
  {
    id_ = srv.response.id;
    state_.state_name_ = "_";

    return true;
  }
  else
    return false;
}

template<class MessageType>
bool StateMachineServer<MessageType>::waitForResult(ros::Duration timeout)
{
  ros::Time strat = ros::Time(0);
  bool end = false;

  while((end == false) && ((timeout == ros::Duration(-1)) || (ros::Time(0) - strat <= timeout)) && ros::ok())
  {
    if(state_.state_name_ == "")
    {
      end = true;
      continue;
    }
  }

  return end;
}

template<class MessageType>
bool StateMachineServer<MessageType>::cancel()
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
void StateMachineServer<MessageType>::init(ros::NodeHandle& n, bool spin_thread)
{
  if(spin_thread)
  {
    ROS_DEBUG("Spinning up a thread for the StateMachineServer");
    need_to_terminate_ = false;
    n.setCallbackQueue(&callback_queue_);
    spin_thread_ = new std::thread(std::bind(&StateMachineServer<MessageType>::spinThread, this));
  }
  else
  {
    spin_thread_ = nullptr;
  }
}

template<class MessageType>
void StateMachineServer<MessageType>::spinThread()
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
void StateMachineServer<MessageType>::statusCallback(const resource_management_msgs::StateMachinesStatus msg)
{
  if((int)msg.id == id_)
  {
    state_.state_name_ = msg.state_name;
    state_.state_event_ = msg.state_event;
    if(status_callback_)
      status_callback_(state_);
  }
}

}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATEMACHINESERVER_H_
