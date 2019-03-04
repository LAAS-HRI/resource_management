#ifndef MESSAGEABSTRACTION_H
#define MESSAGEABSTRACTION_H

#include "message_storage/PriorityHolder.h"

class MessageAbstraction : public PriorityHolder
{
public:
  virtual ~MessageAbstraction() {}

  virtual void publish() = 0;
};

#endif // MESSAGEABSTRACTION_H
