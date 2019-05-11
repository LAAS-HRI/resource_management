#ifndef PRIORITYHOLDER_H
#define PRIORITYHOLDER_H

#include <stdint.h>
#include <vector>

namespace resource_management {

enum importance_priority_t
{
  vital     = 4,
  urgent    = 3,
  high      = 2,
  standard  = 1,
  low       = 0,
  void_msg  = -1
};

enum focus_priority_t
{
  atomic      = 4,
  prioritize  = 3,
  normal      = 2,
  secondary   = 1,
  background  = 0,
  inhibit     = -1
};

template<typename T>
class PriorityHolder
{
public:
  PriorityHolder() {}

  void setPriority(T priority)
  {
    priority_ = priority;
    priorities_ = std::vector<std::vector<int8_t> >(
      {
        {0 , 1 , 2 , 3 , 16},
        {4 , 5 , 6 , 7 , 17},
        {8 , 9 , 10, 11, 18},
        {12, 13, 14, 15, 19},
        {20, 21, 22, 23, 24}
      }
    );
  }
  T getPriority() { return priority_; }

protected:
  T priority_;
  std::vector<std::vector<int8_t> > priorities_;
};

} // namespace resource_management

#endif // PRIORITYHOLDER_H
