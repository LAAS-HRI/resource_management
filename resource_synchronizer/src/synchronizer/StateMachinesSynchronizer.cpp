#include "resource_synchronizer/synchronizer/StateMachinesSynchronizer.h"

namespace resource_synchronizer
{

void StateMachinesSynchronizer::insert(int id, const std::string& resource, std::vector<std::string>& synchros)
{
  if(synchro_holders_.find(id) == synchro_holders_.end())
    synchro_holders_[id] = StateMachineSynchroHolder();

  synchro_holders_[id].insert(resource, synchros);
}

bool StateMachinesSynchronizer::activate(int id, const std::string& synchro, const std::string& resource)
{
  if(synchro_holders_.find(id) != synchro_holders_.end())
    return synchro_holders_[id].activate(synchro, resource);
  else
    return false;
}

void StateMachinesSynchronizer::reset(int id, const std::string& synchro)
{
  if(synchro_holders_.find(id) != synchro_holders_.end())
    synchro_holders_[id].reset(synchro);
}

void StateMachinesSynchronizer::erase(int id)
{
  if(synchro_holders_.find(id) != synchro_holders_.end())
    synchro_holders_.erase(id);
}

} // namespace resource_synchronizer
