#include "resource_management/message_storage/ReactiveBufferStorage.h"

ReactiveBufferStorage::ReactiveBufferStorage(std::vector<std::string> names)
{
  for(const auto& name : names){
    buffers_[name] = std::make_shared<ReactiveBuffer>(name);
  }

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

void ReactiveBufferStorage::setPriority(std::string name, focus_priority_t priority)
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

std::shared_ptr<ReactiveBuffer> ReactiveBufferStorage::getMorePriority()
{
  std::shared_ptr<ReactiveBuffer> found;
  int16_t max_priority = -100;

  for(const auto& it : buffers_)
  {
    if((*it.second)())
    {
      int16_t tmp = (int)priorities_[(int)(*it.second)()->getPriority() + 2][(int)it.second->getPriority()];
      if(tmp > max_priority)
      {
        max_priority = tmp;
        found = it.second;
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
