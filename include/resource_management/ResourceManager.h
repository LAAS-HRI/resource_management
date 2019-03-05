#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_HPP_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_HPP_

#include <vector>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "resource_management/ReactiveInputs.h"
#include "message_storage/ReactiveBuffer.h"

template<class CoordinationSignalMsg, class ... PriorityMsgs>
class ResourceManager
{
    public:
    ResourceManager(ros::NodeHandlePtr nh, const std::vector<ReactiveInputs*> &prio_buffers);

private:

    ros::NodeHandlePtr _nh;

    std::vector<ReactiveInputsBase*> _prioInputs;


};


template<class CoordinationSignalMsg, class ...PriorityMsgs>
ResourceManager<CoordinationSignalMsg,PriorityMsgs...>::ResourceManager(ros::NodeHandlePtr nh, const std::vector<ReactiveInputs *> &prio_buffers):
    _nh(std::move(nh))
{

}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_HPP_
