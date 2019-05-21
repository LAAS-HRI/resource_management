#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESYNCHROHOLDER
#define RESOURCE_SYNCHRONIZER_STATEMACHINESYNCHROHOLDER

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

namespace resource_synchronizer
{

class StateMachineSynchroHolder
{
public:
  void insert(const std::string& resource, std::vector<std::string>& synchros);

  bool activate(const std::string& synchro, const std::string& resource);
  void reset(const std::string& synchro);

private:
  std::map<std::string, std::vector<std::string> > synchros_;
  std::map<std::string, std::vector<bool> > activations_;

  static ros::NodeHandle nh_;
  static std::map<std::string, ros::Publisher> publishers_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESYNCHROHOLDER
