#ifndef COORDINATIONTRANSITION_H
#define COORDINATIONTRANSITION_H

#include <regex>
#include <chrono>
#include <string>

enum transtition_state_t
{
  transition_pass_on_event,
  transition_pass_on_duration,
  transition_timeout,
  transition_wait,
  transition_global_timeout,
  transition_none,
};

class CoordinationTransition
{
public:
  CoordinationTransition(int32_t duration, int32_t time_out, std::vector<std::string> regexs);

  void start();
  void reset();

  transtition_state_t evaluate();
  transtition_state_t evaluate(const std::string& event);

private:
  int32_t duration_;
  int32_t time_out_;
  std::vector<std::regex> regexs_;
  std::vector<bool> regexs_validation_;

  std::chrono::high_resolution_clock::time_point start_;
};

#endif // COORDINATIONTRANSITION_H
