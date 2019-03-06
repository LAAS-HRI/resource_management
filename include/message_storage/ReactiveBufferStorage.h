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
  ~ReactiveBufferStorage() = default;

  void setPriority(std::string name, focus_priority_t priority);
  std::shared_ptr<ReactiveBuffer> operator[](const std::string& name) const;
  std::shared_ptr<ReactiveBuffer> getMorePriority();
  std::shared_ptr<MessageAbstraction> getMorePriorityData();

private:
  std::map<std::string, std::shared_ptr<ReactiveBuffer>> buffers_;
  std::vector<std::vector<int8_t> > priorities_;
};

#endif // REACTIVEBUFFERSTORAGE_H
