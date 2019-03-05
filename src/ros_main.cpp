#include "resource_management/ResourceManager.h"

int main(int argc, char *argv[])
{

    ros::NodeHandlePtr nh(new ros::NodeHandle());
    ResourceManager<float,std::string, bool> rm(nh,{"p1","p2","p3"});

    return 0;
}
