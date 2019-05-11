#include "led_manager_msgs/StateMachineRegister.h"

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

  led_manager_msgs::StateMachineRegister::Request signal;
  signal.header.timeout = ros::Duration(-1);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management_msgs::MessagePriority::URGENT;
  signal.header.initial_state = "state_0";

  led_manager_msgs::StateMachineStateColor color_state;
  led_manager_msgs::StateMachineStateOnOff onoff_state;

  /* STATE_0 */

  onoff_state.data = true;
  onoff_state.header.id = "state_0";
  onoff_state.header.transitions.clear();

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_1";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(10);//1
    onoff_state.header.transitions.push_back(transition);
  }

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_2";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(1);//-1
    transition.end_condition.regex_end_condition.push_back("end_loop");
    onoff_state.header.transitions.push_back(transition);
  }

  signal.state_machine.states_OnOff.push_back(onoff_state);

  /* STATE_1 */

  color_state.data = 100;
  color_state.header.id = "state_1";
  color_state.header.transitions.clear();

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_0";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(1);
    color_state.header.transitions.push_back(transition);
  }

  signal.state_machine.states_Color.push_back(color_state);

  /* STATE_2 */

  color_state.data = 150;
  color_state.header.id = "state_2";
  color_state.header.transitions.clear();

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_3";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    color_state.header.transitions.push_back(transition);
  }

  signal.state_machine.states_Color.push_back(color_state);

  /* STATE_3 */

  color_state.data = 200;
  color_state.header.id = "state_3";
  color_state.header.transitions.clear();

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_4";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    color_state.header.transitions.push_back(transition);
  }

  signal.state_machine.states_Color.push_back(color_state);

  /* STATE_4 */

  onoff_state.data = false;
  onoff_state.header.id = "state_4";
  onoff_state.header.transitions.clear();

  {
    resource_management_msgs::StateMachineTransition transition;
    transition.next_state = "state_5";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    onoff_state.header.transitions.push_back(transition);
  }

  signal.state_machine.states_OnOff.push_back(onoff_state);

  std::cout << "will pub" << std::endl;
  led_manager_msgs::StateMachineRegister srv;
  ros::ServiceClient client = nh->serviceClient<led_manager_msgs::StateMachineRegister>("/led_manager_test/state_machines_register");
  srv.request = signal;
  client.call(srv);

  std::cout << "id = " << srv.response.id << std::endl;

  return 0;
}
