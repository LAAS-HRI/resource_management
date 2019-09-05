#include "led_resource_synchronizer_msgs/MetaStateMachineRegister.h"

#include "resource_synchronizer/tools/MetaStateMachineClient.h"
#include "resource_management/API/StatesMsg.h"

#include <ros/ros.h>

/*
add_executable(${PROJECT_NAME}_test src/server_test.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
*/

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"led_synchronizer_test_pub");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  resource_synchronizer::MetaStateMachineClient<led_resource_synchronizer_msgs::MetaStateMachineRegister> server("led_resource_synchronizer");
  server.waitForServer();

  led_resource_synchronizer_msgs::MetaStateMachineRegister::Request signal;
  signal.header.timeout = ros::Duration(-1);
  signal.header.begin_dead_line = ros::Time::now() + ros::Duration(5);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;

  // CREATE SM //
  {
    led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub;
    sub.header.timeout = ros::Duration(-1);
    sub.header.begin_dead_line = ros::Time::now() + ros::Duration(5);
    sub.header.initial_state = "state_0";

    resource_management::StatesMsg<led_manager_msgs::StateMachineStateOnOff> states_OnOff;
    resource_management::StatesMsg<led_manager_msgs::StateMachineStateColor> states_Color;

    /* STATE_0 */
    states_Color.addState("state_0", 100);
    states_Color["state_0"]->addTransition("state_01", ros::Duration(-1), ros::Duration(1.5));

<<<<<<< HEAD
    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_0", 100);
      state.addTransition("state_01", ros::Duration(-1), ros::Duration(-1));
      sub.state_machine.states_Color.push_back(state());
    }

    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_01", 100);
      state.addTransition("state_1", ros::Duration(-1), ros::Duration(-1), {"__synchro__blip"});
      sub.state_machine.states_Color.push_back(state());
    }
=======
    /* STATE_01 */
    states_Color.addState("state_01", 100);
    states_Color["state_01"]->addTransition("state_1", ros::Duration(-1), ros::Duration(-1), {"__synchro__blip"});
>>>>>>> ec156f54c00740567d7c659c0445c0bf6a41e7d9

    /* STATE_1 */
    states_Color.addState("state_1", 150);
    states_Color["state_1"]->addTransition("state_10", ros::Duration(-1), ros::Duration(1.5));

    states_Color.addState("state_10", 150);
    states_Color["state_10"]->addTransition("state_0", ros::Duration(-1), ros::Duration(-1), {"__synchro__blop"});

    sub.state_machine.states_OnOff = states_OnOff();
    sub.state_machine.states_Color = states_Color();

<<<<<<< HEAD
    signal.state_machine_led_G = sub;
  }

  // CREATE SM //

=======
    signal.state_machine_led_R = sub;
  }

  // CREATE SM //
>>>>>>> ec156f54c00740567d7c659c0445c0bf6a41e7d9
  {
    led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub;
    sub.header.timeout = ros::Duration(-1);
    sub.header.begin_dead_line = ros::Time::now() + ros::Duration(5);
    sub.header.initial_state = "state_0";

    resource_management::StatesMsg<led_manager_msgs::StateMachineStateOnOff> states_OnOff;
    resource_management::StatesMsg<led_manager_msgs::StateMachineStateColor> states_Color;

    /* STATE_0 */
<<<<<<< HEAD

    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_0", 100);
      state.addTransition("state_01", ros::Duration(-1), ros::Duration(1.5));
      sub.state_machine.states_Color.push_back(state());
    }

    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_01", 100);
      state.addTransition("state_1", ros::Duration(-1), ros::Duration(-1), {"__synchro__blip"});
      sub.state_machine.states_Color.push_back(state());
    }

    /* STATE_1 */

    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_1", 150);
      state.addTransition("state_10", ros::Duration(-1), ros::Duration(1.5));
      sub.state_machine.states_Color.push_back(state());
    }

    {
      resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_10", 150);
      state.addTransition("state_0", ros::Duration(-1), ros::Duration(-1), {"__synchro__blop"});
      sub.state_machine.states_Color.push_back(state());
    }

    signal.state_machine_led_R = sub;
=======
    states_Color.addState("state_0", 100);
    states_Color["state_0"]->addTransition("state_01", ros::Duration(-1), ros::Duration(1.5));

    /* STATE_01 */
    states_Color.addState("state_01", 100);
    states_Color["state_01"]->addTransition("state_1", ros::Duration(-1), ros::Duration(-1), {"__synchro__blip"});

    /* STATE_1 */
    states_Color.addState("state_1", 150);
    states_Color["state_1"]->addTransition("state_10", ros::Duration(-1), ros::Duration(1.5));

    states_Color.addState("state_10", 150);
    states_Color["state_10"]->addTransition("state_0", ros::Duration(-1), ros::Duration(-1), {"__synchro__blop"});

    sub.state_machine.states_OnOff = states_OnOff();
    sub.state_machine.states_Color = states_Color();

    signal.state_machine_led_G = sub;
>>>>>>> ec156f54c00740567d7c659c0445c0bf6a41e7d9
  }

  led_resource_synchronizer_msgs::MetaStateMachineRegister srv;
  srv.request = signal;
  server.send(srv);

  if(server.waitForResult())
    std::cout << "SUCCESS" << std::endl;
  else
    std::cout << "ERROR" << std::endl;

  std::cout << server.getResult().toString() << std::endl;

  return 0;
}
