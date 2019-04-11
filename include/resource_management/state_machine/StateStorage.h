#ifndef STATESTORAGE_H
#define STATESTORAGE_H

#include "resource_management/state_machine/CoordinationState.h"
#include "resource_management/message_storage/MessagePriority.h"
#include "resource_management/message_storage/MessageAbstraction.h"

#include <map>
#include <string>

namespace resource_management {

class StateStorage : public MessagePriority
{
public:
  StateStorage(uint32_t id = -1, ros::Duration time_out = ros::Duration(-1), ros::Time begin_dead_line = ros::Time(0));
  ~StateStorage();

  void addState(const std::string& id);
  void addTransition(const std::string& id, const std::string& id_next, CoordinationTransition& transition);
  void addData(const std::string& id, std::shared_ptr<MessageAbstraction> data);

  void setInitialState(const std::string& id);
  CoordinationState* getInitialState();
  std::shared_ptr<MessageAbstraction> getStateData(const std::string& id);

  ros::Duration getTimeout() { return  time_out_; }
  ros::Time getDeadLine() { return  begin_dead_line_; }
  uint32_t getId() { return id_; }
  bool isTooLate();

  CoordinationState* operator[](const std::string& id);
private:
  uint32_t id_;
  std::map<std::string, CoordinationState*> states_;
  std::map<std::string, std::shared_ptr<MessageAbstraction>> datas_;
  std::string initial_state_;

  ros::Duration time_out_;
  ros::Time begin_dead_line_;
};

} // namespace resource_management

#endif // STATESTORAGE_H
