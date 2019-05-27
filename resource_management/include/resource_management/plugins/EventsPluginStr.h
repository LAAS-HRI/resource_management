#ifndef EVENTSPLUGINTEST_H
#define EVENTSPLUGINTEST_H

#include <string>

#include "std_msgs/String.h"

#include "resource_management/plugins/EventsInterface.h"

namespace resource_management
{

class EventsPluginStr : public EventsInterface
{
public:
  EventsPluginStr() {}
  void setNodeHandle(ros::NodeHandlePtr nh);

private:
  ros::Subscriber _subscriber;

  void callback(const std_msgs::String::ConstPtr& msg);
};

} //  namespace resource_management

#endif // EVENTSPLUGINTEST_H
