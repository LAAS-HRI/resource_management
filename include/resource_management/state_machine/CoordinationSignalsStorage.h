#ifndef COORDINATIONSIGNALSSTORAGE_H
#define COORDINATIONSIGNALSSTORAGE_H

#include "resource_management/state_machine/StateStorage.h"

#include <vector>
#include <mutex>

namespace resource_management {

class CoordinationSignalsStorage
{
public:
  bool empty();
  void push(std::shared_ptr<StateStorage>& state_storage);
  std::shared_ptr<StateStorage> pop();
  bool remove(uint32_t id);

private:
  std::vector<std::shared_ptr<StateStorage> > states_storage_;
  std::mutex mutex_;
};

} // namespace resource_management

#endif //COORDINATIONSIGNALSSTORAGE_H
