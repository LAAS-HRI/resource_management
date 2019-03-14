#include "led_manager/CoordinationSignal.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"led_manager_test");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  led_manager::CoordinationSignal::Request signal;
  signal.header.timeout = ros::Duration(-1);
  signal.header.begin_dead_line = ros::Time(0);
  signal.header.priority.value = resource_management::MessagePriority::URGENT;

  led_manager::CoordinationStateColor color_state;
  led_manager::CoordinationStateOnOff onoff_state;

  /* STATE_0 */

  onoff_state.data = true;
  onoff_state.header.id = "state_0";

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_1";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(1);
    onoff_state.header.transitions.push_back(transition);
  }

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_2";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(-1);
    transition.end_condition.regex_end_condition.push_back("end_loop");
    onoff_state.header.transitions.push_back(transition);
  }

  signal.states_OnOff.push_back(onoff_state);

  /* STATE_1 */

  color_state.data = 100;
  color_state.header.id = "state_1";

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_0";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(1);
    color_state.header.transitions.push_back(transition);
  }

  signal.states_Color.push_back(color_state);

  /* STATE_2 */

  color_state.data = 150;
  color_state.header.id = "state_2";

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_3";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    color_state.header.transitions.push_back(transition);
  }

  signal.states_Color.push_back(color_state);

  /* STATE_3 */

  color_state.data = 200;
  color_state.header.id = "state_3";

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_4";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    color_state.header.transitions.push_back(transition);
  }

  signal.states_Color.push_back(color_state);

  /* STATE_4 */

  onoff_state.data = false;
  onoff_state.header.id = "state_4";

  {
    resource_management::CoordinationSignalsTransition transition;
    transition.next_state = "state_5";
    transition.end_condition.timeout = ros::Duration(-1);
    transition.end_condition.duration = ros::Duration(2);
    onoff_state.header.transitions.push_back(transition);
  }

  signal.states_OnOff.push_back(onoff_state);

  std::cout << "will pub" << std::endl;
  led_manager::CoordinationSignal srv;
  ros::ServiceClient client = nh->serviceClient<led_manager::CoordinationSignal>("/led_manager/coordination_signals_register");
  srv.request = signal;
  client.call(srv);

  std::cout << "id = " << srv.response.id << std::endl;

  return 0;
}
