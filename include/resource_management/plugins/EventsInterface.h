#ifndef EVENTSINTERFACE_H
#define EVENTSINTERFACE_H

#include <string>

#include <ros/ros.h>

namespace resource_management
{

class EventsInterface
{
public:
  virtual ~EventsInterface() {}
  
  virtual void setNodeHandle(ros::NodeHandlePtr nh)
  {
    _nh = nh;
  }

  void registerSpreading(std::function<void(const std::string&)> SpreadEvent)
  {
    _SpreadEvent = SpreadEvent;
  }

protected:
  std::function<void(const std::string&)> _SpreadEvent;
  ros::NodeHandlePtr _nh;
};

} //  namespace resource_management

#endif // EVENTSINTERFACE_H
