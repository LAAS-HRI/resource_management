#ifndef REACTIVEBUFFERSTORAGE_H
#define REACTIVEBUFFERSTORAGE_H

#include "message_storage/ReactiveBuffer.h"

#include <vector>
#include <string>
#include <map>

class ReactiveBufferStorage
{
public:
  ReactiveBufferStorage(std::vector<std::string> names);
  ~ReactiveBufferStorage();

  void setPriority(std::string name, uint8_t priority);
  ReactiveBuffer* operator[](const std::string& name);
  ReactiveBuffer* getMorePriority();
  MessageAbstraction* getMorePriorityData();

private:
  std::map<std::string, ReactiveBuffer*> buffers_;
};

#endif // REACTIVEBUFFERSTORAGE_H
