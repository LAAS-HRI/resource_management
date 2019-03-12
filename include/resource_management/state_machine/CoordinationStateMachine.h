#ifndef COORDINATIONSTATEMACHINE_H
#define COORDINATIONSTATEMACHINE_H

#include "resource_management/state_machine/EventStorage.h"

#include "resource_management/state_machine/CoordinationTransition.h"
#include "resource_management/state_machine/CoordinationState.h"

#include <ros/ros.h>

#include <string>
#include <mutex>

namespace resource_management {

struct CoordinationInternalState_t
{
  uint32_t state_machine_id;
  CoordinationState* state_;
  transtition_state_t transition_state_;

  CoordinationInternalState_t()
  {
    state_ = nullptr;
    transition_state_ = transition_none;
  }

  CoordinationInternalState_t(CoordinationState* state, transtition_state_t transition_state)
  {
    state_ = state;
    transition_state_ = transition_state;
  }
};

class CoordinationStateMachine : public EventStorage
{
public:
  CoordinationStateMachine(float rate = 100);

  void run();
  CoordinationInternalState_t getInternalState();
  std::string getCurrentStateName();

  void setInitialState(CoordinationState* state, uint32_t state_machine_id = 0);
  void setPublicationFunction(std::function<void(CoordinationInternalState_t)> publishState);
  void setTimeout(ros::Duration time_out) { time_out_ = time_out; }
  void setDeadLine(ros::Time begin_dead_line) { begin_dead_line_ = begin_dead_line; }

private:
  uint32_t us_sleep_time_;
  ros::Duration time_out_;
  ros::Time begin_dead_line_;

  std::function<void(CoordinationInternalState_t)> publishState_;
  CoordinationInternalState_t internal_state_;
  std::mutex internal_state_mutex_;

  void runOnceNoEvent();
  void runOnceWithEvents(std::queue<std::string>& events);
};

} // namespace resource_management

#endif // COORDINATIONSTATEMACHINE_H
