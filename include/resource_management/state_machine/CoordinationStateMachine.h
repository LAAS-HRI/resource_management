#ifndef COORDINATIONSTATEMACHINE_H
#define COORDINATIONSTATEMACHINE_H

#include "resource_management/state_machine/EventStorage.h"

#include "resource_management/state_machine/StateMachineTransition.h"
#include "resource_management/state_machine/StateMachineState.h"

#include <ros/ros.h>

#include <string>
#include <mutex>

namespace resource_management {

class CoordinationStateMachine : public EventStorage
{
public:
  CoordinationStateMachine(float rate = 100);

  void run();
  StateMachineInternalState_t getInternalState();
  std::string getCurrentStateName();
  bool isNewState();
  bool isWildcardState();
  bool runing();

  void setInitialState(StateMachineState* state, uint32_t state_machine_id = 0);
  void setPublicationFunction(std::function<void(StateMachineInternalState_t)> publishState);
  void setTimeout(ros::Duration time_out) { time_out_ = time_out; }
  void setDeadLine(ros::Time begin_dead_line) { begin_dead_line_ = begin_dead_line; }

private:
  uint32_t us_sleep_time_;
  ros::Duration time_out_;
  ros::Time begin_dead_line_;

  std::function<void(StateMachineInternalState_t)> publishState_;
  StateMachineInternalState_t internal_state_;
  bool new_state_;
  std::mutex internal_state_mutex_;

  void runOnceNoEvent();
  void runOnceWithEvents(std::queue<std::string>& events);
};

} // namespace resource_management

#endif // COORDINATIONSTATEMACHINE_H
