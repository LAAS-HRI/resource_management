#include "message_storage/ReactiveBuffer.h"

ReactiveBuffer::ReactiveBuffer(std::string name)
{
  name_ = name;
  data_ = nullptr;
}

void ReactiveBuffer::setData(MessageAbstraction* data)
{
  data_ = data;
}

void ReactiveBuffer::replaceData(MessageAbstraction* data)
{
  if(data_ != nullptr)
    delete data_;

  data_ = data;
}

MessageAbstraction* ReactiveBuffer::operator()()
{
  return data_;
}
