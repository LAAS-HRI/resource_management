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
  signal.header.timeout = ros::Duration(9);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;

  // CREATE SM //
  {
    led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub;
    sub.header.timeout = ros::Duration(10);
    sub.header.begin_dead_line = ros::Time(0);
    sub.header.initial_state = "state_0";

    resource_management::StatesMsg<led_manager_msgs::StateMachineStateOnOff> states_OnOff;
    resource_management::StatesMsg<led_manager_msgs::StateMachineStateColor> states_Color;

    /* STATE_0 */
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

    signal.state_machine_led_R = sub;
  }

  // CREATE SM //
  {
    led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub;
    sub.header.timeout = ros::Duration(10);
    sub.header.begin_dead_line = ros::Time(0);
    sub.header.initial_state = "state_0";

    resource_management::StatesMsg<led_manager_msgs::StateMachineStateOnOff> states_OnOff;
    resource_management::StatesMsg<led_manager_msgs::StateMachineStateColor> states_Color;

    /* STATE_0 */
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
