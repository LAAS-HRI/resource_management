#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H

#include <vector>

#include "resource_synchronizer/StateMachinesHolder.h"

namespace resource_synchronizer
{

class StateMachinesManager
{
public:
  StateMachinesManager() {}

  void registerHolder(StateMachinesHolderBase* holder)
  {
    state_machines_holders_.push_back(holder);
  }

  void run()
  {
    
  }

private:
  std::vector<StateMachinesHolderBase*> state_machines_holders_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
