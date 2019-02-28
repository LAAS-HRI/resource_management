#ifndef COORDINATIONSTATE_H
#define COORDINATIONSTATE_H

#include "state_machine/CoordinationTransition.h"

#include <string>
#include <vector>

class CoordinationSate
{
public:
  CoordinationSate(std::string id);

  void setTransition(CoordinationSate* next, CoordinationTransition tansition);

  CoordinationSate* update(std::string& event);
  CoordinationSate* update();

private:
  std::string id_;
  std::vector<CoordinationTransition> transitions_conditions_;
  std::vector<CoordinationSate*> transitions_next_state_;

  transtition_state_t internal_state_;
};

#endif // COORDINATIONSTATE_H
