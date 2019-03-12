#ifndef COORDINATIONSTATE_H
#define COORDINATIONSTATE_H

#include "resource_management/state_machine/CoordinationTransition.h"

#include <string>
#include <vector>

class CoordinationState
{
public:
  CoordinationState(std::string id);

  void setTransition(CoordinationState* next, CoordinationTransition tansition);
  std::string getName() { return id_; }

  void startState();
  transtition_state_t update(CoordinationState** current_state, const std::string& event = "");

private:
  std::string id_;
  std::vector<CoordinationTransition> transitions_conditions_;
  std::vector<CoordinationState*> transitions_next_state_;
};

#endif // COORDINATIONSTATE_H
