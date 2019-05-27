#include "${project_name}/${project_name}.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"${project_name}");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    ${class_name} mgr(nh, plugins, true);

    std::thread th(&${class_name}::run, &mgr);

    ros::spin();

    th.join();
}
