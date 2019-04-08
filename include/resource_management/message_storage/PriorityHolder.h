#ifndef PRIORITYHOLDER_H
#define PRIORITYHOLDER_H

#include <stdint.h>
#include <vector>

namespace resource_management {

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
  ignore      = 0,
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
        {-6, -5, -4, -3, -2},
        {-1, 0 , 1 , 2 , 19},
        {3 , 4 , 5 , 6 , 20},
        {7 , 8 , 9 , 10, 21},
        {11, 12, 13, 14, 22},
        {15, 16, 17, 18, 23},
        {24, 25, 26, 27, 28}
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
