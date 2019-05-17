#ifndef ${project_name}_RESOURCE_SYNCHRONIZER_H
#define ${project_name}_RESOURCE_SYNCHRONIZER_H

#include <map>

#include <ros/ros.h>

#include "resource_management_msgs/StateMachinesCancel.h"
#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"

#include "${project_name_msgs}/MetaStateMachine.h"
#include "resource_synchronizer/StateMachinesHolder.h"
#include "resource_synchronizer/StateMachinesManager.h"

!!for dep in unique_msgs_deps
#include "{dep}/StateMachineRegister.h"
!!end

namespace ${project_name}
{

class ${class_name}
{
public:
  ${class_name}(ros::NodeHandlePtr nh);

  void publishStatus(resource_synchronizer::SubStateMachineStatus status);

  void run();

private:
  ros::NodeHandlePtr _nh;
!!for sub_fsm in sub_fsms
  resource_synchronizer::StateMachinesHolder<${{project_name_msgs}}::{sub_fsm.type}, {sub_fsm.res_name}::StateMachineRegister> _holder_{sub_fsm.name};
!!end
  resource_synchronizer::StateMachinesManager _manager;

  unsigned int _current_id;
  std::map<int, resource_synchronizer_msgs::MetaStateMachinesStatus> _status;

  ros::ServiceServer _register_service;
  ros::Publisher _state_machine_status_publisher;
  ros::ServiceServer _state_machine_cancel_service;

  bool registerMetaStateMachine(${project_name_msgs}::MetaStateMachine::Request &req,
                                ${project_name_msgs}::MetaStateMachine::Response &res);

  bool stateMachineCancel
      (resource_management_msgs::StateMachinesCancel::Request  &req,
      resource_management_msgs::StateMachinesCancel::Response &res);

  void removeStatusIfNeeded(int id);
};

} // namespace ${project_name}

#endif // ${project_name}_RESOURCE_SYNCHRONIZER_H
