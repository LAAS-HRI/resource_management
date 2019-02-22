#include "state_machine/CoordinationTransition.h"

CoordinationTransition(int32_t duration, int32_t time_out, std::vector<std::string> regexs)
{
  duration_ = duration;
  time_out_ = time_out;
  for(const auto& regex : regexs)
  {
    regexs_.push_back(std::regex(regex));
    regexs_validation_.push_back(false);
  }
}

void CoordinationTransition::start()
{
  start_ = std::chrono::high_resolution_clock::now();
}

void CoordinationTransition::reset()
{
  start_ = {};
}

transtition_state_t CoordinationTransition::evaluate()
{
  std::chrono::high_resolution_clock::time_point now = high_resolution_clock::now();

  if((time_out_ != -1) && (duration_cast<duration<double>>(now - start_).count >= time_out_))
    return transition_timeout;
  else if((duration_ != 0) && (duration_cast<duration<double>>(now - start_).count >= duration_))
    return transition_pass;
  else
    return transition_wait;
}

transtition_state_t CoordinationTransition::evaluate(std::string event)
{
  std::chrono::high_resolution_clock::time_point now = high_resolution_clock::now();

  if((time_out_ != -1) && (duration_cast<duration<double>>(now - start_).count >= time_out_))
    return transition_timeout;
  else if((duration_ != 0) && (duration_cast<duration<double>>(now - start_).count >= duration_))
    return transition_pass;
  else
    return transition_wait;
}
