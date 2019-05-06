#ifndef MESSAGEPRIORITY_H
#define MESSAGEPRIORITY_H

#include "resource_management/message_storage/PriorityHolder.h"

namespace resource_management {

class MessagePriority : public PriorityHolder<importance_priority_t>
{
public:
  MessagePriority() {};

  int operator*(focus_priority_t other)
  {
    return (int)priorities_[(int)priority_][(int)other];
  }
private:
};

} //namespace resource_management


#endif // MESSAGEPRIORITY_H
