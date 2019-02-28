#include "state_machine/CoordinationStateMachine.h"

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
  CoordinationState s1("state1");
  CoordinationState s2("state2");

  CoordinationTransition t1(1, -1, std::vector<std::string>());
  s1.setTransition(&s2, t1);

  CoordinationStateMachine sm;
  sm.setPublicationFunction(&publishState);
  sm.setInitialState(&s1);

  std::thread th(&CoordinationStateMachine::run, &sm);
  th.join();

  return 0;
}
