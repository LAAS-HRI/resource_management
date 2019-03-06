#ifndef REACTIVEBUFFER_H
#define REACTIVEBUFFER_H

#include "message_storage/PriorityHolder.h"
#include "message_storage/MessageAbstraction.h"

#include <string>
#include <memory>

class ReactiveBuffer : public PriorityHolder<focus_priority_t>
{
public:
  ReactiveBuffer(std::string name);

  std::string getName() { return name_; }

  void setData(std::shared_ptr<MessageAbstraction> data);
  std::shared_ptr<MessageAbstraction> operator()();

private:
  std::string name_;
  std::shared_ptr<MessageAbstraction> data_;
};

#endif // REACTIVEBUFFER_H
