#ifndef COORDINATIONSTATE_H
#define COORDINATIONSTATE_H

#include "resource_management/state_machine/CoordinationTransition.h"

#include <string>
#include <vector>

namespace resource_management {

class CoordinationState;

struct CoordinationInternalState_t
{
  uint32_t state_machine_id;
  CoordinationState* state_;
  transtition_state_t transition_state_;

  CoordinationInternalState_t()
  {
    state_ = nullptr;
    transition_state_ = transition_none;
  }

  CoordinationInternalState_t(CoordinationState* state, transtition_state_t transition_state)
  {
    state_ = state;
    transition_state_ = transition_state;
  }
};

class CoordinationState
{
public:
  CoordinationState(std::string id);

  void setTransition(CoordinationState* next, CoordinationTransition tansition);
  std::string getName() { return id_; }

  void startState();
  transtition_state_t update(CoordinationState** current_state, const std::string& event = "");
  bool endState();

private:
  std::string id_;
  std::vector<CoordinationTransition> transitions_conditions_;
  std::vector<CoordinationState*> transitions_next_state_;
};

} // namespace resource_management

#endif // COORDINATIONSTATE_H
