#include "resource_management/state_machine/StateStorage.h"

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

void StateStorage::addState(const std::string& id)
{
  if(states_.find(id) == states_.end())
    states_[id] = new StateMachineState(id);
}

void StateStorage::addTransition(const std::string& id, const std::string& id_next, StateMachineTransition& transition)
{
  if(states_.find(id) == states_.end())
    addState(id);

  if(states_.find(id_next) == states_.end())
    addState(id_next);

  states_[id]->setTransition(states_[id_next], transition);
}

void StateStorage::addData(const std::string& id, std::shared_ptr<MessageAbstraction> data)
{
  if(states_.find(id) == states_.end())
    addState(id);

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

} // namespace resource_management
