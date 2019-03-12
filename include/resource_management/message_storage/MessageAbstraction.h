#ifndef MESSAGEABSTRACTION_H
#define MESSAGEABSTRACTION_H

#include "resource_management/message_storage/PriorityHolder.h"

namespace resource_management {

class MessageAbstraction : public PriorityHolder<importance_priority_t>
{
public:
  virtual ~MessageAbstraction() {}

  virtual void publish() = 0;
};

} // namespace resource_management

#endif // MESSAGEABSTRACTION_H
