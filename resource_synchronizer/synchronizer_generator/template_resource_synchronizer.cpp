#include "${project_name}.h"

#include <thread>

#include <resource_synchronizer_msgs/MetaStateMachineHeader.h>

namespace ${project_name}
{

${class_name}::${class_name}(ros::NodeHandlePtr nh) : _nh(std::move(nh)),
!!for sub_fsm in sub_fsms
  _holder_{sub_fsm.name}("{sub_fsm.name}"),
!!end
  _current_id(0)
{

  _register_service = _nh->advertiseService("state_machines_register", &${class_name}::registerMetaStateMachine, this);
  _state_machine_status_publisher = _nh->advertise<resource_synchronizer_msgs::MetaStateMachinesStatus>("state_machine_status", 10);
  _state_machine_cancel_service = _nh->advertiseService("state_machine_cancel", &${class_name}::stateMachineCancel, this);

!!for sub_fsm in sub_fsms
  _holder_{sub_fsm.name}.registerSatusCallback([this](auto status){{ this->publishStatus(status); }});
!!end

!!for sub_fsm in sub_fsms
  _manager.registerHolder(&_holder_{sub_fsm.name});
!!end

  ROS_INFO("${project_name} ready.");
}

void ${class_name}::publishStatus(resource_synchronizer_msgs::MetaStateMachinesStatus status)
{
  _state_machine_status_publisher.publish(status);
}

void ${class_name}::run()
{
  std::thread ${project_name}_thread(&resource_synchronizer::StateMachinesManager::run, &_manager);

  ros::spin();

  _manager.stop();
  ${project_name}_thread.join();
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

bool ${class_name}::stateMachineCancel
                    (resource_management_msgs::StateMachinesCancel::Request  &req,
                    resource_management_msgs::StateMachinesCancel::Response &res)
{
  bool done = true;
!!for sub_fsm in sub_fsms
  done = done || _holder_{sub_fsm.name}.cancel(req.id);
!!end

  res.ack = done;

  return true;
}

} // namespace ${project_name}

int main(int argc, char** argv){
  ros::init(argc, argv, "${project_name}");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  ${project_name}::${class_name} syn(nh);

  syn.run();

  return 0;
}
