#include "resource_management/state_machine/StateMachinesStorage.h"

namespace resource_management {

bool StateMachinesStorage::empty()
{
  mutex_.lock();
  size_t nb_states = states_storage_.size();
  mutex_.unlock();

  if(nb_states == 0)
    return true;
  else
    return false;
}

void StateMachinesStorage::push(std::shared_ptr<StateStorage>& state_storage)
{
  mutex_.lock();
  states_storage_.push_back(state_storage);
  mutex_.unlock();
}

std::shared_ptr<StateStorage> StateMachinesStorage::pop(double priority)
{
  int max_index = -1;
  int max_priority = -100;
  std::shared_ptr<StateStorage> res;

  mutex_.lock();
  for(size_t i = 0; i < states_storage_.size(); i++)
    if((int)states_storage_[i]->getPriority() > max_priority)
    {
      max_priority = (int)states_storage_[i]->getPriority();
      max_index = i;
    }

  if(max_index >= 0)
  {
    max_priority = states_storage_[max_index]->operator*(atomic);

    if(max_priority >= priority)
    {
      std::shared_ptr<StateStorage> tmp = states_storage_[max_index];
      states_storage_.erase(states_storage_.begin() + max_index);
      res = tmp;
    }
  }
  mutex_.unlock();

  return res;
}

bool StateMachinesStorage::poppable(double priority)
{
  if(unpoppable_)
  {
    unpoppable_ = false;
    return false;
  }

  int max_index = -1;
  int max_priority = -100;
  std::shared_ptr<StateStorage> res = nullptr;

  mutex_.lock();
  for(size_t i = 0; i < states_storage_.size(); i++)
    if((int)states_storage_[i]->getPriority() > max_priority)
    {
      max_priority = (int)states_storage_[i]->getPriority();
      max_index = i;
    }

  if(max_index >= 0)
  {
    max_priority = states_storage_[max_index]->operator*(atomic);

    if(max_priority > priority)
      res = states_storage_[max_index];
  }
  mutex_.unlock();

  return (res != nullptr);
}

bool StateMachinesStorage::remove(uint32_t id)
{
  bool found = false;

  mutex_.lock();
  for(size_t i = 0; i < states_storage_.size(); )
    if(states_storage_[i]->getId() > id)
    {
      states_storage_.erase(states_storage_.begin() + i);
      found = true;
    }
    else
      i++;
  mutex_.unlock();

  return found;
}

void StateMachinesStorage::setPublicationFunction(std::function<void(CoordinationInternalState_t)> publishState)
{
  publishState_ = publishState;
}

void StateMachinesStorage::clean()
{
  for(size_t i = 0; i < states_storage_.size(); )
  {
    if(states_storage_[i]->isTooLate())
    {
      CoordinationInternalState_t internal_state;
      internal_state.state_machine_id = states_storage_[i]->getId();
      internal_state.state_ = nullptr;
      internal_state.transition_state_ = transition_dead_line;
      // inform that the coordination signal will be removed
      if(publishState_)
        publishState_(internal_state);
      states_storage_.erase(states_storage_.begin() + i);
    }
    else
      i++;
  }
}

} // namespace resource_management
