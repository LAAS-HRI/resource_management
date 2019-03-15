#ifndef MESSAGEWRAPPER_H
#define MESSAGEWRAPPER_H

#include "resource_management/message_storage/MessageAbstraction.h"

#include <functional>

namespace resource_management {

template<typename T>
class MessageWrapper : public MessageAbstraction
{
public:
  MessageWrapper();
  MessageWrapper(const T& data);

  MessageWrapper& operator=(const T& data);
  MessageWrapper& operator=(const MessageWrapper& other);
  T& operator()();

  static void registerPublishFunction(std::function<void(T)> publish);
  void publish();
  std::shared_ptr<MessageAbstraction> clone();

private:
  T data_;
  static std::function<void(T)> publish_;
};

template<typename T>
std::function<void(T)> MessageWrapper<T>::publish_ = {};

template<typename T>
MessageWrapper<T>::MessageWrapper()
{
}

template<typename T>
MessageWrapper<T>::MessageWrapper(const T& data)
{
  data_ = data;
}

template<typename T>
MessageWrapper<T>& MessageWrapper<T>::operator=(const T& data)
{
  data_ = data;
  return *this;
}

template<typename T>
MessageWrapper<T>& MessageWrapper<T>::operator=(const MessageWrapper& other)
{
  if (this != &other)
  {
    data_ = other.data_;
    publish_ = other.publish_;
  }
  return *this;
}

template<typename T>
T& MessageWrapper<T>::operator()()
{
  return data_;
}

template<typename T>
void MessageWrapper<T>::registerPublishFunction(std::function<void(T)> publish)
{
  publish_ = publish;
}

template<typename T>
void MessageWrapper<T>::publish()
{
  if(publish_)
    publish_(data_);
}

template<typename T>
std::shared_ptr<MessageAbstraction> MessageWrapper<T>::clone()
{
  std::shared_ptr<MessageWrapper<T>> tmp = std::make_shared<MessageWrapper<T>>(data_);
  return tmp;
}

} // namespace resource_management

#endif // MESSAGEWRAPPER_H
