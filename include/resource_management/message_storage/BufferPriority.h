#ifndef BUFFERPRIORITY_H
#define BUFFERPRIORITY_H

#include "resource_management/message_storage/PriorityHolder.h"

namespace resource_management {

class BufferPriority : public PriorityHolder<focus_priority_t>
{
public:
  BufferPriority() {};

  int operator*(const importance_priority_t other) const
  {
    return (int)priorities_[(int)other][(int)priority_];
  }
private:
};

} //namespace resource_management


#endif // BUFFERPRIORITY_H
