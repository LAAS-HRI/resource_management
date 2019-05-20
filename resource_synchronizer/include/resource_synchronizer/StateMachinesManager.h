#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H

#include <vector>
#include <atomic>

#include <ros/ros.h>

#include "resource_synchronizer/StateMachinesHolder.h"

namespace resource_synchronizer
{

class StateMachinesManager
{
public:
  StateMachinesManager() : run_(false) {}
  ~StateMachinesManager() {}

  void registerHolder(StateMachinesHolderBase* holder);

  void run();
  void stop();

private:
  std::vector<StateMachinesHolderBase*> state_machines_holders_;
  std::atomic<bool> run_;

  std::vector<bool> done_;
  std::vector<std::vector<int> > ids_;
  std::vector<bool> preempt_;

  void init();
  void reinit();

  bool isDone();

  void clean();
  void cleanPreempted();

  void select(int id);
  void remove(int id);
  bool isSelectable(size_t owner, int id);
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
