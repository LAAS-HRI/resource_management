#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_

#include <vector>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "resource_management/ReactiveInputs.h"
#include "resource_management/CoordinationSignals.h"

#include "message_storage/ReactiveBuffer.h"

class ResourceManager
{
    public:
    ResourceManager(ros::NodeHandlePtr nh, CoordinationSignalsBase *coordinationSignals, std::vector<ReactiveInputsBase*> reactiveInputs);

private:

    ros::NodeHandlePtr _nh;

    CoordinationSignalsBase *_coordinationSignalService;
    std::vector<ReactiveInputsBase*> _reactiveInputs;


};


ResourceManager::ResourceManager(ros::NodeHandlePtr nh, CoordinationSignalsBase *coordinationSignals, std::vector<ReactiveInputsBase *> reactiveInputs):
    _nh(std::move(nh)), _coordinationSignalService(coordinationSignals), _reactiveInputs(reactiveInputs)
{

}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
