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
  void insert(const std::string& resource, const std::vector<std::string>& synchros);

  bool activate(const std::string& synchro, const std::string& resource);
  void reset();

  static void setNodeHandle(ros::NodeHandlePtr nh) { nh_ = nh; }

private:
  std::map<std::string, std::vector<std::string> > synchros_;
  std::map<std::string, std::vector<bool> > activations_;

  static ros::NodeHandlePtr nh_;
  static std::map<std::string, ros::Publisher> publishers_;
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESYNCHROHOLDER
