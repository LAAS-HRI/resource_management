#include "${project_name}_msgs/CoordinationSignal.h"
!!for data_type in message_types
#include "${{project_name}}_msgs/{data_type[0]}.h"
!!end
#include "${project_name}/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class ${class_name} : public resource_management::ResourceManager<${project_name}_msgs::CoordinationSignal
!!for data_type in message_types
      ,${{project_name}}_msgs::{data_type[0]}
!!end
>
{
public:
    ${class_name}(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins):
        ResourceManager (std::move(nh),{${reactive_input_names_cs}}, plugins)
    {
        // this in lambda is necessary for gcc <= 5.1
!!for data_type in message_types
        resource_management::MessageWrapper<{data_type[2]}>::registerPublishFunction([this](auto data, auto is_new){{ this->publish{data_type[0]}Msg(data, is_new); }});
!!end

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<${project_name}::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const ${project_name}_msgs::CoordinationSignal::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const ${project_name}_msgs::CoordinationSignal::Request &msg) override;
    ${project_name}_msgs::CoordinationSignal::Response generateResponseMsg(uint32_t id) override;

!!for data_type in message_types
    void publish{data_type[0]}Msg({data_type[2]} msg, bool is_new);
!!end
};

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> ${class_name}::stateFromMsg(const ${project_name}_msgs::CoordinationSignal::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;
!!for data_type in message_types

    for(auto x : msg.states_{data_type[0]}){{
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<{data_type[2]}>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }}
!!end

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
${class_name}::transitionFromMsg(const ${project_name}_msgs::CoordinationSignal::Request &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;
!!for data_type in message_types

    for(auto x : msg.states_{data_type[0]}){{
        for(auto t : x.header.transitions){{
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }}
    }}
!!end
    return transitions;
}

${project_name}_msgs::CoordinationSignal::Response ${class_name}::generateResponseMsg(uint32_t id)
{
  ${project_name}_msgs::CoordinationSignal::Response res;
  res.id = id;
  return res;
}

!!for data_type in message_types
void ${{class_name}}::publish{data_type[0]}Msg({data_type[2]} msg, bool is_new)
{{
  // Put you own publishing function here
}}

!!end
int main(int argc, char *argv[]){
    ros::init(argc,argv,"${project_name}");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    ${class_name} mgr(nh, plugins);

    std::thread th(&${class_name}::run, &mgr);

    ros::spin();

    th.join();
}
