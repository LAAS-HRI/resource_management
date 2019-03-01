#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"

#include <iostream>
#include <thread>

void publishState(CoordinationInternalState_t state)
{
  std::chrono::time_point<std::chrono::system_clock> now_point = std::chrono::system_clock::now();
  std::time_t now = std::chrono::system_clock::to_time_t(now_point);
  std::cout << "[" << std::ctime(&now) << "] ";

  std::cout << "[STATE] ";
  if(state.state_ != nullptr)
    std::cout << state.state_->getName() << " : ";
  else
    std::cout << "end : ";

  switch (state.transition_state_) {
    case transition_pass_on_event : std::cout << "pass_on_event"; break;
    case transition_pass_on_duration : std::cout << "pass_on_duration"; break;
    case transition_timeout : std::cout << "timeout"; break;
    case transition_wait : std::cout << "wait"; break;
    case transition_global_timeout : std::cout << "global_timeout"; break;
    case transition_none : std::cout << "none"; break;
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  StateStorage states;

  /*states.addState("state1");
  states.addState("state2");
  states.addState("state3");*/

  CoordinationTransition t1(1000, -1, std::vector<std::string>());
  states.addTransition("state1", "state2", t1);

  CoordinationTransition t2(-1, 5000, std::vector<std::string>({"regex"}));
  states.addTransition("state2", "state3", t2);

  CoordinationStateMachine sm;
  sm.setPublicationFunction(&publishState);
  sm.setInitialState(states["state1"]);

  std::thread th(&CoordinationStateMachine::run, &sm);
  th.join();

  return 0;
}
