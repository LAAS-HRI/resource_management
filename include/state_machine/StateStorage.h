#ifndef STATESTORAGE_H
#define STATESTORAGE_H

#include "state_machine/CoordinationState.h"
#include "message_storage/PriorityHolder.h"

#include <map>
#include <string>

class StateStorage : public PriorityHolder<importance_priority_t>
{
public:
  StateStorage(ros::Duration time_out = ros::Duration(-1), ros::Time begin_dead_line = ros::Time(0));
  ~StateStorage();

  void addState(const std::string& id);
  void addTransition(const std::string& id, const std::string& id_next, CoordinationTransition& transition);

  void setInitialState(const std::string& id);
  CoordinationState* getInitialState();

  ros::Duration getTimeout() { return  time_out_; }
  ros::Time getDeadLine() { return  begin_dead_line_; }

  CoordinationState* operator[](std::string id);
private:
  std::map<std::string, CoordinationState*> states_;
  std::string initial_state_;

  ros::Duration time_out_;
  ros::Time begin_dead_line_;
};

#endif // STATESTORAGE_H
