#include "${project_name}.h"

#include <thread>

#include "resource_synchronizer_msgs/MetaStateMachineHeader.h"
#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"

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

void ${class_name}::publishStatus(resource_synchronizer::SubStateMachineStatus status)
{
  auto it = _status.find(status.id);
  if(it != _status.end())
  {
    int sub_id = -1;
    for(size_t i = 0; i < it->second.resource.size(); i++)
      if(it->second.resource[i] == status.resource)
      {
        sub_id = i;
        break;
      }

    if(sub_id != -1)
    {
      it->second.state_name[sub_id] = status.state_name;
      it->second.state_event[sub_id] = status.event_name;
      _state_machine_status_publisher.publish(it->second);
      removeStatusIfNeeded(sub_id);
    }
  }
}

void ${class_name}::run()
{
  std::thread ${project_name}_thread(&resource_synchronizer::StateMachinesManager::run, &_manager);

  ros::spin();

  _manager.stop();
  ${project_name}_thread.join();
}

bool ${class_name}::registerMetaStateMachine(${project_name_msgs}::MetaStateMachineRegister::Request &req,
${project_name_msgs}::MetaStateMachineRegister::Response &res){
  _status[_current_id].id = _current_id;
  bool inserted;

!!for sub_fsm in sub_fsms
  inserted = _holder_{sub_fsm.name}.insert(_current_id, req.state_machine_{sub_fsm.name}, req.header.priority);
  if(inserted)
    _status[_current_id].resource.push_back("{sub_fsm.name}");

!!end
  res.id = _current_id;

  _status[_current_id].state_name.resize(_status[_current_id].resource.size(), "_");
  _status[_current_id].state_event.resize(_status[_current_id].resource.size());
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

void ${class_name}::removeStatusIfNeeded(int id)
{
  bool remove = true;
  for(size_t i = 0; i < _status[id].state_name.size(); i++)
    if(_status[id].state_name[i] != "")
      remove = false;

  if(remove)
    _status.erase(id);
}

} // namespace ${project_name}

int main(int argc, char** argv){
  ros::init(argc, argv, "${project_name}");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  ${project_name}::${class_name} syn(nh);

  syn.run();

  return 0;
}
