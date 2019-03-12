#include "resource_management/state_machine/CoordinationState.h"

#include <iostream>

CoordinationState::CoordinationState(std::string id)
{
  id_ = id;
}

void CoordinationState::setTransition(CoordinationState* next, CoordinationTransition tansition)
{
  transitions_conditions_.push_back(tansition);
  transitions_next_state_.push_back(next);
}

void CoordinationState::startState()
{
  for(auto& transition : transitions_conditions_)
    transition.start();
}

transtition_state_t CoordinationState::update(CoordinationState** current_state, const std::string& event)
{
  transtition_state_t transtition_state = transition_none;
  size_t next_index = -1;

  if(transitions_conditions_.size() == 0)
    *current_state =  nullptr;

  for(size_t i = 0; i < transitions_conditions_.size(); i++)
  {
    if(event == "")
      transtition_state = transitions_conditions_[i].evaluate();
    else
      transtition_state = transitions_conditions_[i].evaluate(event);

    if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    {
      next_index = i;
      break;
    }
    else if(transtition_state == transition_timeout)
      break;
  }

  if((transtition_state == transition_pass_on_event) || (transtition_state == transition_pass_on_duration))
    *current_state = transitions_next_state_[next_index];
  else if(transtition_state == transition_timeout)
    *current_state =  nullptr;
  else if(transtition_state == transition_wait)
    *current_state = this;

  return transtition_state;
}
