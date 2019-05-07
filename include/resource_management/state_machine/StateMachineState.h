#ifndef STATEMACHINESTATE_H
#define STATEMACHINESTATE_H

#include "resource_management/state_machine/StateMachineTransition.h"

#include <string>
#include <vector>

namespace resource_management {

class StateMachineState;

struct StateMachineInternalState_t
{
  uint32_t state_machine_id;
  StateMachineState* state_;
  transtition_state_t transition_state_;

  StateMachineInternalState_t()
  {
    state_ = nullptr;
    transition_state_ = transition_none;
  }

  StateMachineInternalState_t(StateMachineState* state, transtition_state_t transition_state)
  {
    state_ = state;
    transition_state_ = transition_state;
  }
};

class StateMachineState
{
public:
  StateMachineState(std::string id);

  void setTransition(StateMachineState* next, StateMachineTransition tansition);
  std::string getName() { return id_; }

  void startState();
  transtition_state_t update(StateMachineState** current_state, const std::string& event = "");
  bool endState();

private:
  std::string id_;
  std::vector<StateMachineTransition> transitions_conditions_;
  std::vector<StateMachineState*> transitions_next_state_;
};

} // namespace resource_management

#endif // STATEMACHINESTATE_H
