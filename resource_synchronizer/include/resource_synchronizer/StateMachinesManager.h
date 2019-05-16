#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H

#include <vector>
#include <atomic>

#include "resource_synchronizer/StateMachinesHolder.h"

namespace resource_synchronizer
{

class StateMachinesManager
{
public:
  StateMachinesManager() : run_(false) {}
  ~StateMachinesManager() {}

  void registerHolder(StateMachinesHolderBase* holder)
  {
    state_machines_holders_.push_back(holder);
  }

  void run()
  {
    run_ = true;
    while(run_ && ros::ok())
    {

    }
  }

  void stop()
  {
    run_ = false;
  }

private:
  std::vector<StateMachinesHolderBase*> state_machines_holders_;
  std::atomic<bool> run_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
