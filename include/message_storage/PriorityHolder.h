#ifndef PRIORITYHOLDER_H
#define PRIORITYHOLDER_H

#include <stdint.h>

class PriorityHolder
{
public:
  PriorityHolder();

  void setPriority(uint8_t priority);
  uint8_t getPriority();

  bool operator<(const PriorityHolder& other);
  bool operator>(const PriorityHolder& other);
  bool operator==(const PriorityHolder& other);
  bool operator!=(const PriorityHolder& other);
  bool operator<=(const PriorityHolder& other);
  bool operator>=(const PriorityHolder& other);

private:
  uint8_t priority_;
};

#endif // PRIORITYHOLDER_H
