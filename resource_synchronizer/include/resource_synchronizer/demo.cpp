#include "resource_synchronizer/StateMachine.h"

#include "led_resource_synchronizer_msgs/SubStateMachine_led_manager_msgs.h"
#include "resource_management_msgs/MessagePriority.h"

///////////

#include "resource_synchronizer/StateMachinesHolder.h"
#include "led_resource_synchronizer_msgs/SubStateMachine_led_manager_msgs.h"
#include "led_manager_msgs/StateMachineRegister.h"

int main()
{
  led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs sub_st;
  resource_management_msgs::MessagePriority led_priority;

  resource_synchronizer::StateMachine<led_manager_msgs::StateMachine> st(sub_st.state_machine, sub_st.header, led_priority);

  /////

  resource_synchronizer::StateMachinesHolder<led_resource_synchronizer_msgs::SubStateMachine_led_manager_msgs, led_manager_msgs::StateMachineRegister> holder("blop");

  holder.insert(sub_st, led_priority);

  return 0;
}
