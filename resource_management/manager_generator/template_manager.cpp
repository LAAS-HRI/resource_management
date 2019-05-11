#include "${project_name}/${project_name}.h"

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> ${class_name}::stateFromMsg(const ${project_name}_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;
!!for data_type in message_types

    for(auto x : msg.state_machine.states_{data_type[0]}){{
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<{data_type[2]}>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }}
!!end

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
${class_name}::transitionFromMsg(const ${project_name}_msgs::StateMachineRegister::Request &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;
!!for data_type in message_types

    for(auto x : msg.state_machine.states_{data_type[0]}){{
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

${project_name}_msgs::StateMachineRegister::Response ${class_name}::generateResponseMsg(uint32_t id)
{
  ${project_name}_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

!!for data_type in message_types
void ${{class_name}}::publish{data_type[0]}Msg({data_type[2]} msg, bool is_new)
{{
  // Put you own publishing function here
}}

!!end
