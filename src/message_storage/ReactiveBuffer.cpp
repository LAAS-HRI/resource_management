#include "message_storage/ReactiveBuffer.h"

ReactiveBuffer::ReactiveBuffer(std::string name)
{
  name_ = name;
  data_ = nullptr;
}

void ReactiveBuffer::setData(std::shared_ptr<MessageAbstraction> data)
{
  data_ = data;
}

std::shared_ptr<MessageAbstraction> ReactiveBuffer::operator()()
{
  return data_;
}
