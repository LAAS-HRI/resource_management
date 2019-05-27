#include "resource_management/state_machine/StateMachineRunner.h"

#include <unistd.h>

namespace resource_management {

StateMachineRunner::StateMachineRunner(float rate)
{
  publishState_ = nullptr;
  time_out_ = ros::Duration(-1);
  begin_dead_line_ = ros::Time(0);
  us_sleep_time_ = (uint32_t)(1000000.0 / rate);
  new_state_ = false;
}

bool StateMachineRunner::runing()
{
  bool res = false;
  internal_state_mutex_.lock();
  res = (internal_state_.state_ != nullptr);
  internal_state_mutex_.unlock();
  return res;
}

void StateMachineRunner::run()
{
  if(internal_state_.state_ != nullptr)
    internal_state_.state_->startState();

  getEvents();

  ros::Time start_time = ros::Time::now();
  if(begin_dead_line_ != ros::Time(0))
    if(begin_dead_line_ < start_time)
    {
      internal_state_mutex_.lock();
      internal_state_.state_ = nullptr;
      internal_state_.transition_state_ = transition_dead_line;
      new_state_ = true;

      if(publishState_ != nullptr)
        publishState_(internal_state_);

      internal_state_mutex_.unlock();
      return;
    }

  while((internal_state_.state_ != nullptr) && (ros::ok()))
  {
    internal_state_mutex_.lock();

    //tets duration end condition
    ros::Time now = ros::Time::now();
    if((time_out_ != ros::Duration(-1)) && (now - start_time >= time_out_))
    {
      internal_state_.state_ = nullptr;
      internal_state_.transition_state_ = transition_global_timeout;
      new_state_ = true;

      if(publishState_ != nullptr)
        publishState_(internal_state_);

      internal_state_mutex_.unlock();
      break;
    }

    std::queue<std::string> events = getEvents();

    // test preamption
    // queue must be converted into vector to execture a find on it
    std::queue<std::string> events_save = events;
    std::vector<std::string> vect_events;
    while(!events_save.empty())
    {
      vect_events.push_back(events_save.front());
      events_save.pop();
    }
    if(std::find(vect_events.begin(), vect_events.end(), "__preamted__") != vect_events.end())
    {
      internal_state_.state_ = nullptr;
      internal_state_.transition_state_ = transition_preampt;
      new_state_ = true;

      if(publishState_ != nullptr)
        publishState_(internal_state_);

      internal_state_mutex_.unlock();
      break;
    }

    //run current state
    if(events.empty())
      runOnceNoEvent();
    else
      runOnceWithEvents(events);

    internal_state_mutex_.unlock();

    usleep(us_sleep_time_);
  }

  internal_state_mutex_.lock();
  internal_state_.state_ = nullptr;
  internal_state_.transition_state_ = transition_none;
  new_state_ = true;
  internal_state_mutex_.unlock();
}

StateMachineInternalState_t StateMachineRunner::getInternalState()
{
  internal_state_mutex_.lock();
  StateMachineInternalState_t tmp = internal_state_;
  internal_state_mutex_.unlock();
  return tmp;
}

std::string StateMachineRunner::getCurrentStateName()
{
  std::string name = "";
  internal_state_mutex_.lock();
  if(internal_state_.state_ != nullptr)
    name = internal_state_.state_->getName();
  internal_state_mutex_.unlock();
  return name;
}

bool StateMachineRunner::isNewState()
{
  bool res = new_state_;
  new_state_= false;
  return res;
}

bool StateMachineRunner::isWildcardState()
{
  bool res = false;
  internal_state_mutex_.lock();
  if(internal_state_.state_ != nullptr)
  {
    std::string name = internal_state_.state_->getName();
    if((name.size()) && (name[0] == '_'))
      res = true;
  }
  internal_state_mutex_.unlock();
  return res;
}

void StateMachineRunner::setInitialState(StateMachineState* state, uint32_t state_machine_id)
{
  internal_state_mutex_.lock();
  internal_state_.state_machine_id = state_machine_id;
  internal_state_.state_ = state;
  internal_state_.transition_state_ = transition_none;
  new_state_ = true;
  internal_state_mutex_.unlock();
}

void StateMachineRunner::setPublicationFunction(std::function<void(StateMachineInternalState_t)> publishState)
{
  publishState_ = publishState;
}

void StateMachineRunner::runOnceNoEvent()
{
  StateMachineState* tmp_state = internal_state_.state_;
  transtition_state_t tmp_transition = internal_state_.state_->update(&tmp_state);

  internal_state_.synchro_name_ = "";
  if(tmp_transition == transition_wait)
  {
    for(auto x : internal_state_.state_->getSynchroNames())
      internal_state_.synchro_name_ += x + "_";
    if(internal_state_.synchro_name_ != "")
      tmp_transition = transition_wait_synchro;
  }

  if(tmp_state != internal_state_.state_)
  {
    internal_state_.state_ = tmp_state;
    internal_state_.transition_state_ = tmp_transition;
    new_state_ = true;

    if(publishState_)
      publishState_(internal_state_);

    if(internal_state_.state_ != nullptr)
      internal_state_.state_->startState();
  }
  else if(tmp_transition != internal_state_.transition_state_)
  {
    internal_state_.transition_state_ = tmp_transition;

    if(publishState_)
      publishState_(internal_state_);
  }
}

void StateMachineRunner::runOnceWithEvents(std::queue<std::string>& events)
{
  while(events.empty() == false)
  {
    if(internal_state_.state_ == nullptr)
      break;

    std::string event = events.front();
    events.pop();

    StateMachineState* tmp_state = internal_state_.state_;
    transtition_state_t tmp_transition = internal_state_.state_->update(&tmp_state, event);

    internal_state_.synchro_name_ = "";
    if(tmp_transition == transition_wait)
    {
      for(auto x : internal_state_.state_->getSynchroNames())
        internal_state_.synchro_name_ += x + "_";
      if(internal_state_.synchro_name_ != "")
        tmp_transition = transition_wait_synchro;
    }

    if(tmp_state != internal_state_.state_)
    {
      internal_state_.state_ = tmp_state;
      internal_state_.transition_state_ = tmp_transition;
      new_state_ = true;

      if(publishState_)
        publishState_(internal_state_);

      if(internal_state_.state_ != nullptr)
        internal_state_.state_->startState();
    }
    else if(tmp_transition != internal_state_.transition_state_)
    {
      internal_state_.transition_state_ = tmp_transition;

      if(publishState_)
        publishState_(internal_state_);
    }
  }
}

} // namespace resource_management
