#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"
#include "state_machine/CoordinationSignalsStorage.h"

#include <ros/ros.h>

#include <iostream>
#include <thread>
#include <unistd.h>

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
    case transition_preampt : std::cout << "preampt"; break;
    case transition_dead_line : std::cout << "dead_line"; break;
    case transition_none : std::cout << "none"; break;
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"st_demo");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  CoordinationSignalsStorage coordination_signals;

  /**********************/
  std::shared_ptr<StateStorage> states = std::make_shared<StateStorage>(ros::Duration(-1),ros::Time::now());
  states->setPriority(important);

  CoordinationTransition t1(ros::Duration(1), ros::Duration(-1), std::vector<std::string>());
  states->addTransition("state1", "state2", t1);

  CoordinationTransition t2(ros::Duration(-1), ros::Duration(5), std::vector<std::string>({"regex"}));
  states->addTransition("state2", "state3", t2);

  states->setInitialState("state1");

  coordination_signals.push(states);

  /**********************/
  std::shared_ptr<StateStorage> states_2 = std::make_shared<StateStorage>();
  states_2->setPriority(urgent);

  CoordinationTransition t3(ros::Duration(1), ros::Duration(-1), std::vector<std::string>());
  states_2->addTransition("state4", "state5", t3);

  CoordinationTransition t4(ros::Duration(-1), ros::Duration(2), std::vector<std::string>());
  states_2->addTransition("state5", "state6", t4);

  states_2->setInitialState("state4");

  coordination_signals.push(states_2);

  /**********************/
  while(coordination_signals.empty() == false)
  {
    std::cout << " ************* " << std::endl;
    std::shared_ptr<StateStorage> current = coordination_signals.pop();

    CoordinationStateMachine sm;
    sm.setPublicationFunction(std::bind(publishState, std::placeholders::_1));
    sm.setInitialState(current->getInitialState());
    sm.setTimeout(current->getTimeout());
    sm.setDeadLine(current->getDeadLine());

    std::thread th(&CoordinationStateMachine::run, &sm);
    usleep(2000000);
    sm.addEvent("regex");
    th.join();
  }

  return 0;
}
