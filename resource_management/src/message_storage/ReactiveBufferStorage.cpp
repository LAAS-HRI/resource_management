#include "resource_management/message_storage/ReactiveBufferStorage.h"

namespace resource_management {

ReactiveBufferStorage::ReactiveBufferStorage(const std::vector<std::string>& names)
{
  buffers_names_ = names;
  for(const auto& name : buffers_names_){
    buffers_[name] = std::make_shared<ReactiveBuffer>(name);
  }
}

void ReactiveBufferStorage::setPriority(const std::string& name, focus_priority_t priority)
{
  if(buffers_.find(name) != buffers_.end())
    buffers_[name]->setPriority(priority);
}

std::shared_ptr<ReactiveBuffer> ReactiveBufferStorage::operator[](const std::string& name) const
{
  auto search = buffers_.find(name);
  if(search != buffers_.end())
    return search->second;
  return nullptr;
}

double ReactiveBufferStorage::getHighestPriority()
{
  double max_priority = -100;

  for(const auto& it : buffers_names_)
  {
    if((*buffers_[it])())
    {
      if((int)buffers_[it]->getPriority() >= 0) // test if buffer is inhibit
      {
        if((int)(*buffers_[it])()->getPriority() >= 0) // test if message if void
        {
          int16_t tmp = (*buffers_[it])()->operator*(buffers_[it]->getPriority());
          if(tmp > max_priority)
            max_priority = tmp;
        }
      }
    }
  }

  return max_priority;
}

std::shared_ptr<ReactiveBuffer> ReactiveBufferStorage::getMorePriority()
{
  std::shared_ptr<ReactiveBuffer> found;
  int16_t max_priority = -100;

  for(const auto& it : buffers_names_)
  {
    if((*buffers_[it])())
    {
      if((int)buffers_[it]->getPriority() >= 0) // test if buffer is inhibit
      {
        if((int)(*buffers_[it])()->getPriority() >= 0) // test if message if void
        {
          int16_t tmp = (*buffers_[it])()->operator*(buffers_[it]->getPriority());
          if(tmp > max_priority)
          {
            max_priority = tmp;
            found = buffers_[it];
          }
        }
      }
    }
  }

  return found;
}

std::shared_ptr<MessageAbstraction> ReactiveBufferStorage::getMorePriorityData()
{
  std::shared_ptr<ReactiveBuffer> max = getMorePriority();
  if(max)
    return (*max)();
  return {};
}

} // namespace resource_management
