#ifndef PRIORITYHOLDER_H
#define PRIORITYHOLDER_H

#include <stdint.h>

enum importance_priority_t
{
  vital     = 4,
  urgent    = 3,
  important = 2,
  helpful   = 1,
  weak      = 0,
  useless   = -1,
  avoid     = -2
};

enum focus_priority_t
{
  fullfocus   = 4,
  prioritize  = 3,
  normal      = 2,
  secondary   = 1,
  ignore      = 0
};

template<typename T>
class PriorityHolder
{
public:
  PriorityHolder() {}

  void setPriority(T priority) { priority_ = priority; }
  T getPriority() { return priority_; }

private:
  T priority_;
};

#endif // PRIORITYHOLDER_H
