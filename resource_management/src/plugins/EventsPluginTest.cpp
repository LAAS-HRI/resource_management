#include "resource_management/plugins/EventsPluginTest.h"

#include <pluginlib/class_list_macros.h>

namespace resource_management
{

void EventsPluginTest::setNodeHandle(ros::NodeHandlePtr nh)
{
  _nh = nh;
  if(_nh)
    _subscriber = _nh->subscribe("test", 10, &EventsPluginTest::callback, this);
}

void EventsPluginTest::callback(const std_msgs::String::ConstPtr& msg)
{
  if(_SpreadEvent)
    _SpreadEvent(msg->data);
}

} //  namespace resource_management

PLUGINLIB_EXPORT_CLASS(resource_management::EventsPluginTest, resource_management::EventsInterface)
