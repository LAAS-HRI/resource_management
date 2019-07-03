#include "resource_management/state_machine/StateMachineTransition.h"

namespace resource_management {

StateMachineTransition::StateMachineTransition(ros::Duration duration, ros::Duration time_out, const std::vector<std::string>& regexs)
{
  duration_ = duration;
  time_out_ = time_out;
  for(const auto& regex : regexs)
  {
    regexs_.push_back(std::regex(regex));
    regexs_validation_.push_back(false);
    if(regex.find("__synchro__") == 0)
      synchro_names_.push_back(regex.substr(11));
  }
}

void StateMachineTransition::start()
{
  start_ = ros::Time::now();
}

void StateMachineTransition::reset()
{
  start_ = {};
}

transtition_state_t StateMachineTransition::evaluate()
{
  ros::Time now = ros::Time::now();

  if((time_out_ != ros::Duration(-1)) && (now - start_ >= time_out_))
    return transition_timeout;
  else if((duration_ != ros::Duration(-1)) && (now - start_ >= duration_))
    return transition_pass_on_duration;
  else
    return transition_wait;
}

transtition_state_t StateMachineTransition::evaluate(const std::string& event)
{
  ros::Time now = ros::Time::now();

  if((time_out_ != ros::Duration(-1)) && (now - start_ >= time_out_))
    return transition_timeout;
  else if((duration_ != ros::Duration(-1)) && (now - start_ >= duration_))
    return transition_pass_on_duration;
  else
  {
    bool pass = true;

    if(regexs_validation_.size() == 0)
      pass = false;

    for(size_t i = 0; i < regexs_validation_.size(); i++)
    {
      std::smatch match;
      if(std::regex_match(event, match, regexs_[i]))
        regexs_validation_[i] = true;
      else if(regexs_validation_[i] == false)
        pass = false;
    }

    if(pass == true)
      return transition_pass_on_event;
    else
      return transition_wait;
  }
}

void StateMachineTransition::analyse(const std::string& state_name)
{
  if(time_out_ == ros::Duration(0))
    std::cout << "\t\t[WARNING] state " << state_name << ": transition timeout set to 0" << std::endl;

  if(duration_ == ros::Duration(0))
    std::cout << "\t\t[WARNING] state " << state_name << ": transition duration set to 0" << std::endl;
}

} // namespace resource_management
