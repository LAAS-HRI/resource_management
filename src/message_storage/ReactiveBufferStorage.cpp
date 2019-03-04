#include "message_storage/ReactiveBufferStorage.h"

ReactiveBufferStorage::ReactiveBufferStorage(std::vector<std::string> names)
{
  for(const auto& name : names)
    buffers_[name] = new ReactiveBuffer(name);
}

ReactiveBufferStorage::~ReactiveBufferStorage()
{
  for(const auto& it : buffers_)
    delete it.second;
}

void ReactiveBufferStorage::setPriority(std::string name, uint8_t priority)
{
  if(buffers_.find(name) != buffers_.end())
    buffers_[name]->setPriority(priority);
}

ReactiveBuffer* ReactiveBufferStorage::operator[](const std::string& name)
{
  if(buffers_.find(name) != buffers_.end())
    return buffers_[name];
  return nullptr;
}

ReactiveBuffer* ReactiveBufferStorage::getMorePriority()
{
  ReactiveBuffer* found = nullptr;
  int16_t max_priority = -1;

  for(const auto& it : buffers_)
  {
    int16_t tmp = it.second->getPriority() * (1 + (float)((*it.second)()->getPriority() / 255.0f));
    if(tmp > max_priority)
    {
      max_priority = tmp;
      found = it.second;
    }
  }

  return found;
}

MessageAbstraction* ReactiveBufferStorage::getMorePriorityData()
{
  ReactiveBuffer* max = getMorePriority();
  if(max != nullptr)
    return (*max)();
  return nullptr;
}
