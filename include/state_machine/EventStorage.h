#ifndef EVENTSTORAGE_H
#define EVENTSTORAGE_H

#include <mutex>
#include <string>
#include <queue>

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

#endif // EVENTSTORAGE_H
