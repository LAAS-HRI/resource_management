#ifndef ${project_name}_RESOURCE_SYNCHRONIZER_H
#define ${project_name}_RESOURCE_SYNCHRONIZER_H

#include "${project_name_msgs}/MetaStateMachine.h"
#include "resource_synchronizer/StateMachinesHolder.h"
#include <ros/ros.h>
!!for dep in unique_msgs_deps
#include "{dep}/StateMachineRegister.h"
!!end

namespace ${project_name} {

class ${class_name}
{
public:
  ${class_name}(ros::NodeHandlePtr nh);
  bool registerMetaStateMachine(${project_name_msgs}::MetaStateMachine::Request &req,
                                ${project_name_msgs}::MetaStateMachine::Response &res);

private:
ros::NodeHandlePtr _nh;
!!for sub_fsm in sub_fsms
    resource_synchronizer::StateMachinesHolder<${{project_name_msgs}}::{sub_fsm.type}, {sub_fsm.res_name}::StateMachineRegister> _holder_{sub_fsm.name};
!!end
unsigned int _current_id;

ros::ServiceServer _register_service;
};

} // namespace ${project_name}

#endif // ${project_name}_RESOURCE_SYNCHRONIZER_H
