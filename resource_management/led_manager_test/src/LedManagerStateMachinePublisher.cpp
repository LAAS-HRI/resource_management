#include "led_manager_msgs/StateMachineRegister.h"

#include "resource_management/API/StateMachineClient.h"
#include "resource_management/API/StatesMsg.h"

#include <ros/ros.h>

/*
add_executable(${PROJECT_NAME}_test src/LedManagerStateMachinePublisher.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
*/

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"led_manager_test_pub");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();

  resource_management::StateMachineClient<led_manager_msgs::StateMachineRegister> server("led_manager_test");
  server.waitForServer();

  led_manager_msgs::StateMachineRegister::Request signal;
  signal.header.timeout = ros::Duration(-1);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;
  signal.header.initial_state = "state_0";

  resource_management::StatesMsg<led_manager_msgs::StateMachineStateOnOff> states_OnOff;
  resource_management::StatesMsg<led_manager_msgs::StateMachineStateColor> states_Color;

  /* STATE_0 */
  states_OnOff.addState("state_0", true);
  states_OnOff["state_0"]->addTransition("state_1", ros::Duration(-1), ros::Duration(10));
  states_OnOff["state_0"]->addTransition("state_2", ros::Duration(-1), ros::Duration(1), {"end_loop"});

  /* STATE_1 */
  states_Color.addState("state_1", 100);
  states_Color["state_1"]->addTransition("state_0", ros::Duration(-1), ros::Duration(1));

  /* STATE_2 */
  states_Color.addState("state_2", 150);
  states_Color["state_2"]->addTransition("state_3", ros::Duration(-1), ros::Duration(2));

  /* STATE_3 */
  states_Color.addState("state_3", 200);
  states_Color["state_3"]->addTransition("state_4", ros::Duration(-1), ros::Duration(2));

  /* STATE_4 */
  states_OnOff.addState("state_4", false);
  states_OnOff["state_4"]->addTransition("state_5", ros::Duration(-1), ros::Duration(2));

  signal.state_machine.states_OnOff = states_OnOff();
  signal.state_machine.states_Color = states_Color();

  led_manager_msgs::StateMachineRegister srv;
  srv.request = signal;
  server.send(srv);

  if(server.waitForResult())
    std::cout << "SUCCESS" << std::endl;
  else
    std::cout << "ERROR" << std::endl;

  std::cout << server.getResult().toString() << std::endl;

  return 0;
}
