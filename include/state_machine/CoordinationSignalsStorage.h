#ifndef COORDINATIONSIGNALSSTORAGE_H
#define COORDINATIONSIGNALSSTORAGE_H

#include "state_machine/StateStorage.h"

#include <vector>

class CoordinationSignalsStorage
{
public:
  bool empty();
  void push(std::shared_ptr<StateStorage>& state_storage);
  std::shared_ptr<StateStorage> pop();

private:
  std::vector<std::shared_ptr<StateStorage> > states_storage_;
};

#endif //COORDINATIONSIGNALSSTORAGE_H
