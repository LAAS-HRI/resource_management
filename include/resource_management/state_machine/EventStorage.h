#ifndef EVENTSTORAGE_H
#define EVENTSTORAGE_H

#include <mutex>
#include <string>
#include <queue>

namespace resource_management {

class EventStorage
{
public:
  EventStorage();

  void addEvent(const std::string& event);
  std::queue<std::string> getEvents();

private:
  std::mutex mutex_;

  bool queue_choice_;
  std::queue<std::string> fifo_1;
  std::queue<std::string> fifo_2;
};

} // namespace resource_management

#endif // EVENTSTORAGE_H
