#ifndef STATEMACHINETRANSITION_H
#define STATEMACHINETRANSITION_H

#include <ros/ros.h>

#include <regex>
#include <chrono>
#include <string>

namespace resource_management {

enum transtition_state_t
{
  transition_pass_on_event,
  transition_pass_on_duration,
  transition_timeout,
  transition_wait,
  transition_wait_synchro,
  transition_global_timeout,
  transition_preampt,
  transition_dead_line,
  transition_none,
};

class StateMachineTransition
{
public:
  StateMachineTransition(ros::Duration duration, ros::Duration time_out, const std::vector<std::string>& regexs);

  void start();
  void reset();

  transtition_state_t evaluate();
  transtition_state_t evaluate(const std::string& event);

  std::vector<std::string> getSynchroNames() { return synchro_names_; }

  void analyse(const std::string& state_name);

private:
  ros::Duration duration_;
  ros::Duration time_out_;
  std::vector<std::regex> regexs_;
  std::vector<bool> regexs_validation_;
  std::vector<std::string> synchro_names_;

  ros::Time start_;
};

} // namespace resource_management

#endif // STATEMACHINETRANSITION_H
