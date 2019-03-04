#include "message_storage/PriorityHolder.h"

PriorityHolder::PriorityHolder()
{
  priority_ = 0;
}

void PriorityHolder::setPriority(uint8_t priority)
{
  priority_ = priority;
}

uint8_t PriorityHolder::getPriority()
{
  return priority_;
}

bool PriorityHolder::operator<(const PriorityHolder& other)
{
  if(priority_ < other.priority_)
    return true;
  return false;
}

bool PriorityHolder::operator>(const PriorityHolder& other)
{
  if(priority_ > other.priority_)
    return true;
  return false;
}

bool PriorityHolder::operator==(const PriorityHolder& other)
{
  if(priority_ == other.priority_)
    return true;
  return false;
}

bool PriorityHolder::operator!=(const PriorityHolder& other)
{
  if(priority_ != other.priority_)
    return true;
  return false;
}

bool PriorityHolder::operator<=(const PriorityHolder& other)
{
  if(priority_ <= other.priority_)
    return true;
  return false;
}

bool PriorityHolder::operator>=(const PriorityHolder& other)
{
  if(priority_ >= other.priority_)
    return true;
  return false;
}
