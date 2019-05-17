#include "led_resource_synchronizer_msgs/MetaStateMachineRegister.h"

#include "resource_synchronizer/tools/MetaStateMachineServer.h"
#include "resource_management/tools/StateMsg.h"

#include <ros/ros.h>

/*
add_executable(${PROJECT_NAME}_test src/LedManagerStateMachinePublisher.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
*/

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"led_synchronizer_test_pub");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  resource_management::MetaStateMachineServer<led_resource_synchronizer_msgs::MetaStateMachineRegister> server("led_resource_synchronizer");
  server.waitForServer();

  led_resource_synchronizer_msgs::MetaStateMachineRegister::Request signal;
  signal.header.timeout = ros::Duration(5);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;


  // CREATE SM //

  led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub;
  sub.header.timeout = ros::Duration(10);
  sub.header.begin_dead_line = ros::Time(0);
  sub.header.initial_state = "state_0";

  led_manager_msgs::StateMachineStateColor color_state;

  /* STATE_0 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_0", 100);
    state.addTransition("state_1", ros::Duration(-1), ros::Duration(1.5));
    sub.state_machine.states_Color.push_back(state());
  }

  /* STATE_1 */

  {
    resource_management::StateMsg<led_manager_msgs::StateMachineStateColor, int> state("state_1", 150);
    state.addTransition("state_0", ros::Duration(-1), ros::Duration(1.5));
    sub.state_machine.states_Color.push_back(state());
  }

  signal.state_machine_led_R = sub;
  signal.state_machine_led_G = sub;

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
