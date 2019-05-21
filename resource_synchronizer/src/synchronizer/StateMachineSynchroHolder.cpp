#include "resource_synchronizer/synchronizer/StateMachineSynchroHolder.h"

namespace resource_synchronizer
{

void StateMachineSynchroHolder::insert(const std::string& resource, std::vector<std::string>& synchros)
{
  for(const auto& synchro : synchros)
  {
    synchros_[synchro].push_back(resource);
    activations_[synchro].push_back(false);
  }
}

bool StateMachineSynchroHolder::activate(const std::string& synchro, const std::string& resource)
{
  bool res = false;

  auto it = synchros_.find(synchro);
  if(it != synchros_.end())
  {
    res = true;

    for(size_t i = 0; i < it->second.size(); i++)
    {
      if(it->second[i] == resource)
        activations_[synchro][i] = true;

      res = res && activations_[synchro][i];
    }
  }

  return res;
}

void StateMachineSynchroHolder::reset(const std::string& synchro)
{
  auto it = activations_.find(synchro);
  if(it != activations_.end())
  {
    for(size_t i = 0; i < it->second.size(); i++)
      it->second[i] = false;
  }
}

} // namespace resource_synchronizer
