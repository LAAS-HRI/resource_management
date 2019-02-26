#include "state_machine/CoordinationState.h"

CoordinationSate::CoordinationSate(std::string id)
{
  id_ = id;
}

void CoordinationSate::setTransition(CoordinationSate* next, CoordinationTransition tansition)
{
  transitions_conditions_.push_back(tansition);
  transitions_next_state_.push_back(next);
}
