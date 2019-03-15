#include "led_manager_test/CoordinationSignal.h"
#include "led_manager_test/Color.h"
#include "led_manager_test/OnOff.h"
#include "led_manager_test/ArtificialLife.h"

#include <resource_management/CoordinationSignals.h>
#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class LedManager : public resource_management::ResourceManager<led_manager_test::CoordinationSignal
      ,led_manager_test::Color
      ,led_manager_test::OnOff
>
{
public:
    LedManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins):
        ResourceManager (std::move(nh),{"emotion", "tagada", "switch"}, plugins)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<float>::registerPublishFunction([this](auto data){ this->publishColorMsg(data); });
        resource_management::MessageWrapper<bool>::registerPublishFunction([this](auto data){ this->publishOnOffMsg(data); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<led_manager_test::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const led_manager_test::CoordinationSignal::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
    transitionFromMsg(const led_manager_test::CoordinationSignal::Request &msg) override;
    led_manager_test::CoordinationSignal::Response generateResponseMsg(uint32_t id) override;

    void publishColorMsg(float msg);
    void publishOnOffMsg(bool msg);
};

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> LedManager::stateFromMsg(const led_manager_test::CoordinationSignal::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.states_Color){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<float>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.states_OnOff){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<bool>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
LedManager::transitionFromMsg(const led_manager_test::CoordinationSignal::Request &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>> transitions;

    for(auto x : msg.states_Color){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management::EndCondition(t.end_condition)));
        }
    }

    for(auto x : msg.states_OnOff){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management::EndCondition(t.end_condition)));
        }
    }
    return transitions;
}

led_manager_test::CoordinationSignal::Response LedManager::generateResponseMsg(uint32_t id)
{
  led_manager_test::CoordinationSignal::Response res;
  res.id = id;
  return res;
}

void LedManager::publishColorMsg(float msg)
{
  if(msg < 10)
    std::cout << ' ' << "\r" << std::flush;
  else if(msg > 244)
    std::cout << '@' << "\r" << std::flush;
  else if(msg < 50)
    std::cout << '.' << "\r" << std::flush;
  else if(msg < 100)
    std::cout << '_' << "\r" << std::flush;
  else if(msg < 150)
    std::cout << 'i' << "\r" << std::flush;
  else if(msg < 200)
    std::cout << 'I' << "\r" << std::flush;
  else if(msg < 245)
    std::cout << '0' << "\r" << std::flush;
}

void LedManager::publishOnOffMsg(bool msg)
{
  if(msg)
    std::cout << ' ' << "\r" << std::flush;
  else
    std::cout << '@' << "\r" << std::flush;
}

int main(int argc, char *argv[]){
    ros::init(argc,argv,"led_manager_test");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    LedManager mgr(nh, plugins);

    std::thread th(&LedManager::run, &mgr);

    ros::spin();

    th.join();

    return 0;
}
