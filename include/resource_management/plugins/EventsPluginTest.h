#ifndef EVENTSPLUGINTEST_H
#define EVENTSPLUGINTEST_H

#include <string>
#include <ros/ros.h>

#include "std_msgs/String.h"

#include "resource_management/plugins/EventsInterface.h"

namespace resource_management
{

class EventsPluginTest : public EventsInterface
{
public:
  EventsPluginTest();

private:
  ros::NodeHandlePtr _nh;
  ros::Subscriber _subscriber;

  void callback(const std_msgs::String::ConstPtr& msg);
};

} //  namespace resource_management

#endif // EVENTSPLUGINTEST_H
