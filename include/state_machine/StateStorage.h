#ifndef STATESTORAGE_H
#define STATESTORAGE_H

#include "state_machine/CoordinationState.h"

#include <map>
#include <string>

class StateStorage
{
public:
  ~StateStorage();

  void addState(const std::string& id);
  void addTransition(const std::string& id, const std::string& id_next, CoordinationTransition& transition);

  CoordinationState* operator[](std::string id);
private:
  std::map<std::string, CoordinationState*> states_;
};

#endif // STATESTORAGE_H
