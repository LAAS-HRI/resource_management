#ifndef MESSAGEWRAPPER_H
#define MESSAGEWRAPPER_H

template<typename T>
class MessageWrapper
{
public:
  MessageWrapper();
  MessageWrapper(const T& data);

  MessageWrapper& operator=(const T& data);
  MessageWrapper& operator=(const MessageWrapper& other);
  T& operator()();

  void registerPublishFunction(void (*publish)(T));
  void publish();

private:
  T data_;
  void (*publish_)(T);
};

template<typename T>
MessageWrapper<T>::MessageWrapper()
{
  publish_ = nullptr;
}

template<typename T>
MessageWrapper<T>::MessageWrapper(const T& data)
{
  publish_ = nullptr;
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
void MessageWrapper<T>::registerPublishFunction(void (*publish)(T))
{
  publish_ = publish;
}

template<typename T>
void MessageWrapper<T>::publish()
{
  if(publish_ != nullptr)
    publish_(data_);
}

#endif // MESSAGEWRAPPER_H
