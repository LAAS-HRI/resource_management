#include "resource_synchronizer/synchronizer/StateMachineSynchroHolder.h"

#include "std_msgs/String.h"

namespace resource_synchronizer
{

ros::NodeHandle StateMachineSynchroHolder::nh_;
std::map<std::string, ros::Publisher> StateMachineSynchroHolder::publishers_;

void StateMachineSynchroHolder::insert(const std::string& resource, const std::vector<std::string>& synchros)
{
  for(const auto& synchro : synchros)
  {
    synchros_[synchro].push_back(resource);
    activations_[synchro].push_back(false);
  }
}

bool StateMachineSynchroHolder::activate(const std::string& synchro, const std::string& resource)
{
  bool res = false;

  auto it = synchros_.find(synchro);
  if(it != synchros_.end())
  {
    res = true;

    for(size_t i = 0; i < it->second.size(); i++)
    {
      if(it->second[i] == resource)
        activations_[synchro][i] = true;

      res = res && activations_[synchro][i];
    }

    if(res)
    {
      for(size_t i = 0; i < it->second.size(); i++)
      {
        if(publishers_.find(it->second[i]) == publishers_.end())
          publishers_[it->second[i]] = nh_.advertise<std_msgs::String>(it->second[i] + "/str_events", 10);

        std_msgs::String msg;
        msg.data = "__synchro__" + it->first;
        publishers_[it->second[i]].publish(msg);
      }
    }
  }

  return res;
}

void StateMachineSynchroHolder::reset(const std::string& synchro)
{
  auto it = activations_.find(synchro);
  if(it != activations_.end())
  {
    for(size_t i = 0; i < it->second.size(); i++)
      it->second[i] = false;
  }
}

} // namespace resource_synchronizer
