#ifndef REACTIVEBUFFER_H
#define REACTIVEBUFFER_H

#include "message_storage/PriorityHolder.h"
#include "message_storage/MessageAbstraction.h"

#include <string>

class ReactiveBuffer : public PriorityHolder
{
public:
  ReactiveBuffer(std::string name);

  std::string getName() { return name_; }

  void setData(MessageAbstraction* data);
  MessageAbstraction* operator()();

private:
  std::string name_;
  MessageAbstraction* data_;
};

#endif // REACTIVEBUFFER_H
