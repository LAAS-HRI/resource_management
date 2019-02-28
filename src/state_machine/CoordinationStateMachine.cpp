#include "state_machine/CoordinationStateMachine.h"

#include <unistd.h>

CoordinationStateMachine::CoordinationStateMachine(float rate)
{
  publishState_ = nullptr;
  us_sleep_time_ = (uint32_t)(1000000.0 / rate);
}

void CoordinationStateMachine::run()
{
  if(internal_state_.state_ != nullptr)
    internal_state_.state_->startState();

  while(internal_state_.state_ != nullptr)
  {
    CoordinationState* tmp_state = nullptr;
    transtition_state_t tmp_transition = internal_state_.state_->update(tmp_state);

    internal_state_mutex_.lock();
    if(tmp_state != internal_state_.state_)
    {
      internal_state_.state_ = tmp_state;
      internal_state_.transition_state_ = tmp_transition;

      if(publishState_ != nullptr)
        publishState_(internal_state_);

      internal_state_.state_->startState();
    }
    else if(tmp_transition != internal_state_.transition_state_)
    {
      internal_state_.transition_state_ = tmp_transition;

      if(publishState_ != nullptr)
        publishState_(internal_state_);
    }
    internal_state_mutex_.unlock();

    usleep(us_sleep_time_);
  }
}

CoordinationInternalState_t CoordinationStateMachine::getInternalState()
{
  internal_state_mutex_.lock();
  CoordinationInternalState_t tmp = internal_state_;
  internal_state_mutex_.unlock();
  return tmp;
}
