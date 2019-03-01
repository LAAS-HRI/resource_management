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

CoordinationState* StateStorage::operator[](std::string id)
{
  if(states_.find(id) != states_.end())
    return states_[id];
  else
    return nullptr;
}
