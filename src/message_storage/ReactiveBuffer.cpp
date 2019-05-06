#include "resource_management/message_storage/ReactiveBuffer.h"

namespace resource_management {

ReactiveBuffer::ReactiveBuffer(const std::string& name)
{
  name_ = name;
  data_ = nullptr;
  has_been_published_ = true;
  setPriority(background);
}

void ReactiveBuffer::setData(std::shared_ptr<MessageAbstraction> data)
{
  data_ = std::move(data);
  has_been_published_ = false;
}

std::shared_ptr<MessageAbstraction> ReactiveBuffer::getData()
{
  return data_;
}

std::shared_ptr<MessageAbstraction> ReactiveBuffer::operator()()
{
  return data_;
}

} // namespace resource_management
