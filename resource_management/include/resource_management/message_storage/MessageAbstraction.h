#ifndef MESSAGEABSTRACTION_H
#define MESSAGEABSTRACTION_H

#include <memory>

#include "resource_management/message_storage/MessagePriority.h"

namespace resource_management {

class MessageAbstraction : public MessagePriority
{
public:
  virtual ~MessageAbstraction() {}

  virtual void publish(bool is_new = true) = 0;
  virtual std::shared_ptr<MessageAbstraction> clone() = 0;
};

} // namespace resource_management

#endif // MESSAGEABSTRACTION_H
