#include "${project_name}.h"
#include <resource_synchronizer_msgs/MetaStateMachineHeader.h>

namespace ${project_name} {

${class_name}::${class_name}(ros::NodeHandlePtr nh) : _nh(std::move(nh)),
!!for sub_fsm in sub_fsms
    _holder_{sub_fsm.name}("{sub_fsm.name}"),
!!end
_current_id(0)
{

  _register_service = new ros::ServiceServer(_nh->advertiseService("${project_name}_register_meta_state_machine", &${class_name}::registerMetaStateMachine, this));
  ROS_INFO("${project_name} ready.");
}

bool ${class_name}::registerMetaStateMachine(${project_name_msgs}::MetaStateMachine::Request &req,
${project_name_msgs}::MetaStateMachine::Response &res){
!!for sub_fsm in sub_fsms
  _holder_{sub_fsm.name}.insert(_current_id, req.state_machine_{sub_fsm.name}, req.header.priority);
!!end
  res.id = _current_id;
  _current_id++;
  return true;
}
} // namespace ${project_name}

int main(int argc, char** argv){
  ros::init(argc, argv, "${project_name}");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  ${project_name}::${class_name} syn(nh);

  ros::spin();
}