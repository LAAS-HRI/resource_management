#ifndef EVENTSINTERFACE_H
#define EVENTSINTERFACE_H

#include <string>

namespace resource_management
{

class EventsInterface
{
public:
  void registerSpreading(std::function<void(std::string)> SpreadEvent)
  {
    _SpreadEvent = SpreadEvent;
  }

private:
  std::function<void(std::string)> _SpreadEvent;
};

} //  namespace resource_management

#endif // EVENTSINTERFACE_H
