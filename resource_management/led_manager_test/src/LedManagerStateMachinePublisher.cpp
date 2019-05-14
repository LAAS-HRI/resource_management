#include "led_manager_msgs/StateMachineRegister.h"

#include "resource_management/tools/StateMachineServer.h"
#include "resource_management/tools/StateMsg.h"

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

  resource_management::StateMachineServer<led_manager_msgs::StateMachineRegister> server("led_manager_test");
  server.waitForServer();

  led_manager_msgs::StateMachineRegister::Request signal;
  signal.header.timeout = ros::Duration(-1);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;
  signal.header.initial_state = "state_0";

  led_manager_msgs::StateMachineStateColor color_state;
  led_manager_msgs::StateMachineStateOnOff onoff_state;

  /* STATE_0 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateOnOff, bool> state("state_0", true);
    state.addTransition("state_1", ros::Duration(-1), ros::Duration(10));
    state.addTransition("state_2", ros::Duration(-1), ros::Duration(1), {"end_loop"});
    signal.state_machine.states_OnOff.push_back(state());
  }

  /* STATE_1 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_1", 100);
    state.addTransition("state_0", ros::Duration(-1), ros::Duration(1));
    signal.state_machine.states_Color.push_back(state());
  }

  /* STATE_2 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_2", 150);
    state.addTransition("state_3", ros::Duration(-1), ros::Duration(2));
    signal.state_machine.states_Color.push_back(state());
  }

  /* STATE_3 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_3", 200);
    state.addTransition("state_4", ros::Duration(-1), ros::Duration(2));
    signal.state_machine.states_Color.push_back(state());
  }

  /* STATE_4 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateOnOff, bool> state("state_4", false);
    state.addTransition("state_5", ros::Duration(-1), ros::Duration(2));
    signal.state_machine.states_OnOff.push_back(state());
  }

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
