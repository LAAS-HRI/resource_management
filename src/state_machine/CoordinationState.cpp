#include "state_machine/CoordinationState.h"

#include <iostream>

CoordinationSate::CoordinationSate(std::string id)
{
  id_ = id;
  internal_state_ = transition_none;
}

void CoordinationSate::setTransition(CoordinationSate* next, CoordinationTransition tansition)
{
  transitions_conditions_.push_back(tansition);
  transitions_next_state_.push_back(next);
}

CoordinationSate* CoordinationSate::update(std::string& event)
{
  transtition_state_t transtition_state = transition_none;
  size_t next_index = -1;

  for(size_t i = 0; i < transitions_conditions_.size(); i++)
  {
    transtition_state = transitions_conditions_[i].evaluate();
    if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    {
      next_index = i;
      break;
    }
    else if(transtition_state == transition_timeout)
      break;
  }

  if(transtition_state != internal_state_)
    std::cout << "state change" << std::endl;
  internal_state_ = transtition_state;
  //TODO publish state

  if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    return transitions_next_state_[next_index];
  else if(transtition_state == transition_timeout)
    return nullptr;
  else
    return this;
}

CoordinationSate* CoordinationSate::update()
{
  transtition_state_t transtition_state = transition_none;
  size_t next_index = -1;

  for(size_t i = 0; i < transitions_conditions_.size(); i++)
  {
    transtition_state = transitions_conditions_[i].evaluate();
    if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    {
      next_index = i;
      break;
    }
    else if(transtition_state == transition_timeout)
      break;
  }

  if(transtition_state != internal_state_)
    std::cout << "state change" << std::endl;
  internal_state_ = transtition_state;
  //TODO publish state

  if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    return transitions_next_state_[next_index];
  else if(transtition_state == transition_timeout)
    return nullptr;
  else
    return this;
}
