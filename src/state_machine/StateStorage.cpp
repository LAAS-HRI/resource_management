#include "state_machine/StateStorage.h"

StateStorage::~StateStorage()
{
  for(auto it : states_)
    delete it.second;
}

void StateStorage::addState(const std::string& id)
{
  if(states_.find(id) == states_.end())
    states_[id] = new CoordinationState(id);
}

void StateStorage::addTransition(const std::string& id, const std::string& id_next, CoordinationTransition& transition)
{
  if(states_.find(id) == states_.end())
    addState(id);

  if(states_.find(id_next) == states_.end())
    addState(id_next);

  states_[id]->setTransition(states_[id_next], transition);
}

void StateStorage::setInitialState(const std::string& id)
{
  initial_state_ = id;
}

CoordinationState* StateStorage::getInitialState()
{
  if(states_.find(initial_state_) != states_.end())
    return states_[initial_state_];
  else
    return nullptr;
}

CoordinationState* StateStorage::operator[](std::string id)
{
  if(states_.find(id) != states_.end())
    return states_[id];
  else
    return nullptr;
}
