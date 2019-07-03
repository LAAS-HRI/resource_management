#include "resource_management/state_machine/StateStorage.h"

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif

namespace resource_management {

StateStorage::StateStorage(uint32_t id, ros::Duration time_out, ros::Time begin_dead_line)
{
  id_ = id;
  time_out_ = time_out;
  begin_dead_line_ = begin_dead_line;
}

StateStorage::~StateStorage()
{
  for(auto it : states_)
    delete it.second;
}

void StateStorage::addState(const std::string& id, bool partially_defined)
{
  if(states_.find(id) == states_.end())
    states_[id] = new StateMachineState(id, partially_defined);
  else if(partially_defined == false)
    states_[id]->setAsDefined();
}

void StateStorage::addTransition(const std::string& id, const std::string& id_next, StateMachineTransition& transition)
{
  if(states_.find(id) == states_.end())
    addState(id, true);

  if(states_.find(id_next) == states_.end())
    addState(id_next, true);

  states_[id]->setTransition(states_[id_next], transition);
}

void StateStorage::addData(const std::string& id, std::shared_ptr<MessageAbstraction> data)
{
  if(states_.find(id) == states_.end())
    addState(id, true);

  datas_[id] = data;
}

void StateStorage::setInitialState(const std::string& id)
{
  initial_state_ = id;
}

StateMachineState* StateStorage::getInitialState()
{
  if(states_.find(initial_state_) != states_.end())
    return states_[initial_state_];
  else
    return nullptr;
}

std::shared_ptr<MessageAbstraction> StateStorage::getStateData(const std::string& id)
{
  if(datas_.find(id) != datas_.end())
    return datas_[id];
  else
    return nullptr;
}

StateMachineState* StateStorage::operator[](const std::string& id)
{
  if(states_.find(id) != states_.end())
    return states_[id];
  else
    return nullptr;
}


bool StateStorage::isTooLate()
{
  if((begin_dead_line_ != ros::Time(0)) && (begin_dead_line_ < ros::Time::now()))
    return true;
  else
    return false;
}

void StateStorage::analyse()
{
  std::cout << COLOR_ORANGE;
  if(time_out_ == ros::Duration(0))
    std::cout << "[WARNING] state machine " << id_ << ": timeout to 0" << std::endl;

  if(initial_state_ == "")
    std::cout << "[WARNING] state machine " << id_ << ": initial state has no value" << std::endl;

  if(getInitialState() == nullptr)
    std::cout << "[WARNING] state machine " << id_ << ": initial state is not defined" << std::endl;

  for(auto state : states_)
  {
    if(state.second->isPartiallyDefined())
      std::cout << "[WARNING] state machine " << id_ << ": state " << state.second->getName() << " is not defined" << std::endl;

    if(datas_.find(state.second->getName()) == datas_.end())
      std::cout << "[WARNING] state machine " << id_ << ": state " << state.second->getName() << " do not have data" << std::endl;

    state.second->analyse();
  }

  std::cout << COLOR_OFF;
}

} // namespace resource_management
