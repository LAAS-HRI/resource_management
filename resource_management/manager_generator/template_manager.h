#include "${project_name}_msgs/StateMachineRegister.h"
#include "${project_name}_msgs/StateMachineExtract.h"
!!for data_type in message_types
#include "${{project_name}}_msgs/{data_type[0]}.h"
!!end
#include "${project_name}/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class ${class_name} : public resource_management::ResourceManager<${project_name}_msgs::StateMachineRegister
      ,${project_name}_msgs::StateMachineExtract
!!for data_type in message_types
      ,${{project_name}}_msgs::{data_type[0]}
!!end
>
{
public:
    ${class_name}(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{${reactive_input_names_cs}}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
!!for data_type in message_types
        resource_management::MessageWrapper<{data_type[2]}>::registerPublishFunction([this](auto data, auto is_new){{ this->publish{data_type[0]}Msg(data, is_new); }});
!!end

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<${project_name}::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const ${project_name}_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const ${project_name}_msgs::StateMachine &msg) override;
    ${project_name}_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

!!for data_type in message_types
    void publish{data_type[0]}Msg({data_type[2]} msg, bool is_new);
!!end
};
