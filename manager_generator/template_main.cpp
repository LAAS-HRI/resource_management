#include "${project_name}/CoordinationSignal.h"
!!for msg_name in message_names
#include "${{project_name}}/{msg_name}.h"
!!end

#include <resource_management/CoordinationSignals.h>
#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

class ${class_name} : public ResourceManager<${project_name}::CoordinationSignal
!!for msg_name in message_names
      , ${{project_name}}::{msg_name}
!!end
  >
{
public:
    ${class_name}(const ros::NodeHandlePtr &nh):
        ResourceManager (std::move(nh),{${reactive_input_names_cs}})
    {}

private:
    std::map<std::string,std::shared_ptr<MessageAbstraction>> stateFromMsg(const ${project_name}::CoordinationSignal &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
    transitionFromMsg(const ${project_name}::CoordinationSignal &msg) override;
};

std::map<std::string,std::shared_ptr<MessageAbstraction>> ${class_name}::stateFromMsg(const ${project_name}::CoordinationSignal &msg)
{
    std::map<std::string,std::shared_ptr<MessageAbstraction>> states;
!!for data_type in messages_types_zip
    for(auto x : msg.states_{data_type[0]}){{
        auto wrap = states[x.header.id] = std::make_shared<MessageWrapper<{data_type[2]}>>(x.data);
        wrap->setPriority(static_cast<importance_priority_t>(msg.header.priority.value));
    }}
!!end
    return states;
}
std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
${class_name}::transitionFromMsg(const ${project_name}::CoordinationSignal &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>> transitions;
!!for data_type in message_names
    for(auto x : msg.states_{data_type}){{
        for(auto t : x.header.transitions){{
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management::EndCondition(t.end_condition)));
        }}
    }}
!!end
    return transitions;
}

int main(int argc, char *argv[]){
    ros::init(argc,argv,"${project_name}");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    ${class_name} mgr(nh);

    ros::spin();
}
