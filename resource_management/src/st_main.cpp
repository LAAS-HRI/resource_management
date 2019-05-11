#include "resource_management/state_machine/StateMachineRunner.h"
#include "resource_management/state_machine/StateStorage.h"
#include "resource_management/state_machine/StateMachinesStorage.h"

#include <ros/ros.h>

#include <iostream>
#include <thread>
#include <unistd.h>

void publishState(resource_management::StateMachineInternalState_t state)
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
    case resource_management::transition_pass_on_event : std::cout << "pass_on_event"; break;
    case resource_management::transition_pass_on_duration : std::cout << "pass_on_duration"; break;
    case resource_management::transition_timeout : std::cout << "timeout"; break;
    case resource_management::transition_wait : std::cout << "wait"; break;
    case resource_management::transition_global_timeout : std::cout << "global_timeout"; break;
    case resource_management::transition_preampt : std::cout << "preampt"; break;
    case resource_management::transition_dead_line : std::cout << "dead_line"; break;
    case resource_management::transition_none : std::cout << "none"; break;
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"st_demo");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  resource_management::StateMachinesStorage state_machines;

  /**********************/
  std::shared_ptr<resource_management::StateStorage> states = std::make_shared<resource_management::StateStorage>(0, ros::Duration(-1),ros::Time::now());
  states->setPriority(resource_management::high);

  resource_management::StateMachineTransition t1(ros::Duration(1), ros::Duration(-1), std::vector<std::string>());
  states->addTransition("state1", "state2", t1);

  resource_management::StateMachineTransition t2(ros::Duration(-1), ros::Duration(5), std::vector<std::string>({"regex"}));
  states->addTransition("state2", "state3", t2);

  states->setInitialState("state1");

  state_machines.push(states);

  /**********************/
  std::shared_ptr<resource_management::StateStorage> states_2 = std::make_shared<resource_management::StateStorage>(1);
  states_2->setPriority(resource_management::urgent);

  resource_management::StateMachineTransition t3(ros::Duration(1), ros::Duration(-1), std::vector<std::string>());
  states_2->addTransition("state4", "state5", t3);

  resource_management::StateMachineTransition t4(ros::Duration(-1), ros::Duration(2), std::vector<std::string>());
  states_2->addTransition("state5", "state6", t4);

  states_2->setInitialState("state4");

  state_machines.push(states_2);

  /**********************/
  while(state_machines.empty() == false)
  {
    std::cout << " ************* " << std::endl;
    std::shared_ptr<resource_management::StateStorage> current = state_machines.pop();

    resource_management::StateMachineRunner sm;
    sm.setPublicationFunction(&publishState);
    sm.setInitialState(current->getInitialState(), current->getId());
    sm.setTimeout(current->getTimeout());
    sm.setDeadLine(current->getDeadLine());

    std::thread th(&resource_management::StateMachineRunner::run, &sm);
    usleep(2000000);
    sm.addEvent("regex");
    th.join();
  }

  return 0;
}
