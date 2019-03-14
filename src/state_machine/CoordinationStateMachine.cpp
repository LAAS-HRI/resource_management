#include "resource_management/state_machine/CoordinationStateMachine.h"

#include <unistd.h>

namespace resource_management {

CoordinationStateMachine::CoordinationStateMachine(float rate)
{
  publishState_ = nullptr;
  time_out_ = ros::Duration(-1);
  begin_dead_line_ = ros::Time(0);
  us_sleep_time_ = (uint32_t)(1000000.0 / rate);
}

bool CoordinationStateMachine::runing()
{
  bool res = false;
  internal_state_mutex_.lock();
  res = (internal_state_.state_ != nullptr);
  internal_state_mutex_.unlock();
  return res;
}

void CoordinationStateMachine::run()
{
  if(internal_state_.state_ != nullptr)
    internal_state_.state_->startState();

  ros::Time start_time = ros::Time::now();
  if(begin_dead_line_ != ros::Time(0))
    if(begin_dead_line_ < start_time)
    {
      internal_state_mutex_.lock();
      internal_state_.state_ = nullptr;
      internal_state_.transition_state_ = transition_dead_line;

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

      if(publishState_ != nullptr)
        publishState_(internal_state_);

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

      if(publishState_ != nullptr)
        publishState_(internal_state_);

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
  internal_state_mutex_.unlock();
}

CoordinationInternalState_t CoordinationStateMachine::getInternalState()
{
  internal_state_mutex_.lock();
  CoordinationInternalState_t tmp = internal_state_;
  internal_state_mutex_.unlock();
  return tmp;
}

std::string CoordinationStateMachine::getCurrentStateName()
{
  std::string name = "";
  internal_state_mutex_.lock();
  if(internal_state_.state_ != nullptr)
    name = internal_state_.state_->getName();
  internal_state_mutex_.unlock();
  return name;
}

void CoordinationStateMachine::setInitialState(CoordinationState* state, uint32_t state_machine_id)
{
  internal_state_mutex_.lock();
  internal_state_.state_machine_id = state_machine_id;
  internal_state_.state_ = state;
  internal_state_.transition_state_ = transition_none;
  internal_state_mutex_.unlock();
}

void CoordinationStateMachine::setPublicationFunction(std::function<void(CoordinationInternalState_t)> publishState)
{
  publishState_ = publishState;
}

void CoordinationStateMachine::runOnceNoEvent()
{
  CoordinationState* tmp_state = internal_state_.state_;
  transtition_state_t tmp_transition = internal_state_.state_->update(&tmp_state);

  if(tmp_state != internal_state_.state_)
  {
    internal_state_.state_ = tmp_state;
    internal_state_.transition_state_ = tmp_transition;

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

void CoordinationStateMachine::runOnceWithEvents(std::queue<std::string>& events)
{
  while(events.empty() == false)
  {
    if(internal_state_.state_ == nullptr)
      break;

    std::string event = events.front();
    events.pop();

    CoordinationState* tmp_state = internal_state_.state_;
    transtition_state_t tmp_transition = internal_state_.state_->update(&tmp_state, event);

    if(tmp_state != internal_state_.state_)
    {
      internal_state_.state_ = tmp_state;
      internal_state_.transition_state_ = tmp_transition;

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
