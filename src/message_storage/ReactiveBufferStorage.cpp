#include "message_storage/ReactiveBufferStorage.h"

ReactiveBufferStorage::ReactiveBufferStorage(std::vector<std::string> names)
{
  for(const auto& name : names)
    buffers_[name] = new ReactiveBuffer(name);

  priorities_ = std::vector<std::vector<int8_t> >(
    {
      {-9, -8, -7, -6, -5},
      {-4, -3, -2, -1, 0 },
      {1 , 2 , 3 , 4 , 17},
      {5 , 6 , 7 , 8 , 18},
      {9 , 10, 11, 12, 19},
      {13, 14, 15, 16, 20},
      {21, 22, 23, 24, 25}
    }
  );
}

ReactiveBufferStorage::~ReactiveBufferStorage()
{
  for(const auto& it : buffers_)
    delete it.second;
}

void ReactiveBufferStorage::setPriority(std::string name, focus_priority_t priority)
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
  int16_t max_priority = -100;

  for(const auto& it : buffers_)
  {
    int16_t tmp = (int)priorities_[(int)(*it.second)()->getPriority() + 2][(int)it.second->getPriority()];
    if(tmp > max_priority)
    {
      max_priority = tmp;
      found = it.second;
    }
  }

  return found;
}

std::shared_ptr<MessageAbstraction> ReactiveBufferStorage::getMorePriorityData()
{
  ReactiveBuffer* max = getMorePriority();
  if(max != nullptr)
    return (*max)();
  return nullptr;
}
