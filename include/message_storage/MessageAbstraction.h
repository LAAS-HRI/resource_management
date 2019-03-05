#ifndef MESSAGEABSTRACTION_H
#define MESSAGEABSTRACTION_H

#include "message_storage/PriorityHolder.h"

class MessageAbstraction : public PriorityHolder<importance_priority_t>
{
public:
  virtual ~MessageAbstraction() {}

  virtual void publish() = 0;
};

#endif // MESSAGEABSTRACTION_H
