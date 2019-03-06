#include "state_machine/CoordinationSignalsStorage.h"

bool CoordinationSignalsStorage::empty()
{
  if(states_storage_.size() == 0)
    return true;
  else
    return false;
}

void CoordinationSignalsStorage::push(std::shared_ptr<StateStorage>& state_storage)
{
  states_storage_.push_back(state_storage);
}
#include <iostream>
std::shared_ptr<StateStorage> CoordinationSignalsStorage::pop()
{
  int max_index = -1;
  int max_priority = -100;

  for(size_t i = 0; i < states_storage_.size(); i++)
    if((int)states_storage_[i]->getPriority() > max_priority)
    {
      max_priority = (int)states_storage_[i]->getPriority();
      max_index = i;
    }

  if(max_index >= 0)
  {
    std::shared_ptr<StateStorage> tmp = states_storage_[max_index];
    states_storage_.erase(states_storage_.begin() + max_index);
    return tmp;
  }
  else
    return std::make_shared<StateStorage>();
}
