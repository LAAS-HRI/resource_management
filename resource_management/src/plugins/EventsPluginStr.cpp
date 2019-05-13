#include "resource_management/plugins/EventsPluginStr.h"

#include <pluginlib/class_list_macros.h>

namespace resource_management
{

void EventsPluginStr::setNodeHandle(ros::NodeHandlePtr nh)
{
  _nh = nh;
  if(_nh)
    _subscriber = _nh->subscribe("test", 10, &EventsPluginStr::callback, this);
}

void EventsPluginStr::callback(const std_msgs::String::ConstPtr& msg)
{
  if(_SpreadEvent)
    _SpreadEvent(msg->data);
}

} //  namespace resource_management

PLUGINLIB_EXPORT_CLASS(resource_management::EventsPluginStr, resource_management::EventsInterface)
