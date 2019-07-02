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
  _state_machine_status_publisher = _nh->advertise<resource_synchronizer_msgs::MetaStateMachinesStatus>("state_machine_status", 100);
  _state_machine_cancel_service = _nh->advertiseService("state_machine_cancel", &${class_name}::stateMachineCancel, this);

!!for sub_fsm in sub_fsms
  _holder_{sub_fsm.name}.registerSatusCallback([this](auto status){{ this->publishStatus(status); }});
!!end

!!for sub_fsm in sub_fsms
  _manager.registerHolder(&_holder_{sub_fsm.name});
!!end
  _manager.registerSatusCallback([this](auto status){ this->publishStatus(status); });

  ROS_INFO("${project_name} ready.");
}

void ${class_name}::publishStatus(resource_synchronizer::SubStateMachineStatus status)
{
  mutex_.lock();
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
      if(it->second.state_name[sub_id] != status.state_name)
        _synchronizer.reset(status.id);
      auto tmp = getSynchros(status.event_name);
      for(auto s : tmp)
        _synchronizer.activate(status.id, s, status.resource);

      it->second.state_name[sub_id] = status.state_name;
      it->second.state_event[sub_id] = status.event_name;
      _state_machine_status_publisher.publish(it->second);

      removeStatusIfNeeded(status.id);
    }
  }
  mutex_.unlock();
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

  _manager.halt();

  _status[_current_id].id = _current_id;
  bool inserted;

!!for sub_fsm in sub_fsms
  inserted = _holder_{sub_fsm.name}.insert(_current_id, req.state_machine_{sub_fsm.name}, req.header.priority);
  if(inserted)
  {{
    _status[_current_id].resource.push_back("{sub_fsm.name}");
    _synchronizer.insert(_current_id, "{sub_fsm.name}", _holder_{sub_fsm.name}.getSynchros(_current_id));
  }}

!!end
  _status[_current_id].resource.push_back("_");
  _manager.insert(_current_id, req.header);

  res.id = _current_id;

  _status[_current_id].state_name.resize(_status[_current_id].resource.size(), "_");
  _status[_current_id].state_name[_status[_current_id].state_name.size() - 1] = ""; // global status as no state
  _status[_current_id].state_event.resize(_status[_current_id].resource.size());

  std::cout << "[" << ros::this_node::getName() << "] register " << _current_id << "; " << _status.size() << " meta state machines waiting" << std::endl;

  _current_id++;

  _manager.realease();

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

  _status.erase(req.id);
  res.ack = done;

  return true;
}

void ${class_name}::removeStatusIfNeeded(int id)
{
  mutex_.lock();
  bool need_remove = true;
  for(size_t i = 0; i < _status[id].state_name.size(); i++)
    if(_status[id].state_name[i] != "")
      need_remove = false;

  if(_status[id].state_event.size())
    if(_status[id].state_event[_status[id].state_event.size() - 1] != "")
      need_remove = true;

  if(need_remove)
  {
    _status.erase(id);
    std::cout << "[" << ros::this_node::getName() << "] remove " << id << "; " << _status.size() << " meta state machines waiting" << std::endl;
  }
  mutex_.unlock();
}

std::vector<std::string> ${class_name}::getSynchros(std::string event)
{
  std::vector<std::string> res;

  if(event.find("wait_synchro_") == 0)
  {
    event = event.substr(13);
    res = split(event, "_");
  }

  return res;
}

std::vector<std::string> ${class_name}::split(const std::string& str, const std::string& delim)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
      pos = str.find(delim, prev);
      if (pos == std::string::npos)
        pos = str.length();

      std::string token = str.substr(prev, pos-prev);

      if (!token.empty())
        tokens.push_back(token);
      prev = pos + delim.length();
    }
    while ((pos < str.length()) && (prev < str.length()));

    return tokens;
}

} // namespace ${project_name}

int main(int argc, char** argv){
  ros::init(argc, argv, "${project_name}");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  resource_synchronizer::StateMachineSynchroHolder::setNodeHandle(nh);

  ${project_name}::${class_name} syn(nh);

  syn.run();

  return 0;
}
