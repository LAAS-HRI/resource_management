#include "state_machine/CoordinationStateMachine.h"

#include <unistd.h>

CoordinationStateMachine::CoordinationStateMachine(ros::Duration time_out, float rate)
{
  publishState_ = nullptr;
  time_out_ = time_out;
  us_sleep_time_ = (uint32_t)(1000000.0 / rate);
}

void CoordinationStateMachine::run()
{
  if(internal_state_.state_ != nullptr)
    internal_state_.state_->startState();

  ros::Time start_time = ros::Time::now();

  while(internal_state_.state_ != nullptr)
  {
    internal_state_mutex_.lock();

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

void CoordinationStateMachine::setInitialState(CoordinationState* state)
{
  internal_state_mutex_.lock();
  internal_state_.state_ = state;
  internal_state_.transition_state_ = transition_none;
  internal_state_mutex_.unlock();
}

void CoordinationStateMachine::setPublicationFunction(void (*publishState)(CoordinationInternalState_t))
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

    if(publishState_ != nullptr)
      publishState_(internal_state_);

    if(internal_state_.state_ != nullptr)
      internal_state_.state_->startState();
  }
  else if(tmp_transition != internal_state_.transition_state_)
  {
    internal_state_.transition_state_ = tmp_transition;

    if(publishState_ != nullptr)
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

      if(publishState_ != nullptr)
        publishState_(internal_state_);

      if(internal_state_.state_ != nullptr)
        internal_state_.state_->startState();
    }
    else if(tmp_transition != internal_state_.transition_state_)
    {
      internal_state_.transition_state_ = tmp_transition;

      if(publishState_ != nullptr)
        publishState_(internal_state_);
    }
  }
}
