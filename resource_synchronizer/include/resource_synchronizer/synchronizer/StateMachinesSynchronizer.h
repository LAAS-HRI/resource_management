#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESSYNCHRONIZER
#define RESOURCE_SYNCHRONIZER_STATEMACHINESSYNCHRONIZER

#include <map>

#include "resource_synchronizer/synchronizer/StateMachineSynchroHolder.h"

namespace resource_synchronizer
{

class StateMachinesSynchronizer
{
public:
  void insert(int id, const std::string& resource, const std::vector<std::string>& synchros);

  bool activate(int id, const std::string& synchro, const std::string& resource);
  void reset(int id);

  void erase(int id);

private:
  std::map<int, StateMachineSynchroHolder> synchro_holders_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESSYNCHRONIZER
