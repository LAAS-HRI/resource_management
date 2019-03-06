#include "resource_management/ResourceManager.h"
#include "resource_management/CoordinationSignals.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"ros_demo");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> prio_buffer_names = {"p1","p2","p3"};

    auto stringInputs = new ReactiveInputs<std_msgs::String>(nh,prio_buffer_names);
    auto boolInputs = new ReactiveInputs<std_msgs::Bool>(nh,prio_buffer_names);

//    ResourceManager rm(nh,{{stringInputs,boolInputs}});

    ros::spin();
    return 0;
}
