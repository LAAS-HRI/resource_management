#include "resource_management/state_machine/EventStorage.h"

EventStorage::EventStorage()
{
  queue_choice_ = true;
}


void EventStorage::addEvent(const std::string& event)
{
  mutex_.lock();
  if(queue_choice_ == true)
    fifo_1.push(event);
  else
    fifo_2.push(event);
  mutex_.unlock();
}

std::queue<std::string> EventStorage::getEvents()
{
  std::queue<std::string> tmp;
  mutex_.lock();
  if(queue_choice_ == true)
  {
    while(!fifo_2.empty())
      fifo_2.pop();
    queue_choice_ = false;
    tmp = fifo_1;
  }
  else
  {
    while(!fifo_1.empty())
      fifo_1.pop();
    queue_choice_ = true;
    tmp = fifo_2;
  }
  mutex_.unlock();
  return tmp;
}
