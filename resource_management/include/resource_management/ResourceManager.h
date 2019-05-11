#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_

#include <vector>
#include <string>
#include <utility>
#include <thread>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "std_msgs/String.h"
#include "resource_management/ReactiveInputs.h"
#include "resource_management/StateMachines.h"
#include "resource_management_msgs/PrioritiesSetter.h"
#include "resource_management_msgs/StateMachinesStatus.h"
#include "resource_management_msgs/StateMachinesCancel.h"

#include "resource_management/message_storage/ReactiveBuffer.h"

#include "resource_management/artificial_life/ArtificialLife.h"

#include "resource_management/plugins/EventsInterface.h"

namespace resource_management {

template<typename ...Types>
struct Impl;

template<typename First, typename ...Types>
struct Impl<First, Types...>
{
    template<typename STLStorage, typename ...Args>
    static void add(STLStorage &storage, Args&... args){
        storage.emplace_back(std::make_shared<ReactiveInputs<First>>(args...));
        Impl<Types...>::add(storage,args...);
    }
};
template<>
struct Impl<>
{
    template<typename STLStorage, typename ...Args>
    static void add(STLStorage &, Args&...){}
};

template<typename StateMachineType, typename ...InputDataTypes>
class ResourceManager
{
public:
    /// also creates 2 buffers for artificial life and state machines
    ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames, const std::vector<std::string>& pluginsNames, bool synchronized = false);
    ~ResourceManager();

    void run();

protected:
    virtual std::map<std::string,std::shared_ptr<MessageAbstraction>> stateFromMsg(const typename StateMachineType::Request &msg) = 0;
    virtual std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
        transitionFromMsg(const typename StateMachineType::Request &msg) = 0;
    virtual typename StateMachineType::Response generateResponseMsg(uint32_t id) = 0;

    void done();

    std::shared_ptr<ArtificialLife> _artificialLife;
    std::shared_ptr<ReactiveBuffer> _artificialLifeBuffer;

private:

    const std::vector<std::string> &getBufferNames() const;
    void addBufferNames(const std::vector<std::string> &bufferNames);

    /// initializes buffer storage
    /// also clear _reactiveInputs and _reactiveBuffers
    void createReactiveBufferStorage();

    void prioritiesCallback(const resource_management_msgs::PrioritiesSetter& msg);
    void publishState(StateMachineInternalState_t state);
    bool stateMachineCancel(resource_management_msgs::StateMachinesCancel::Request  &req,
                            resource_management_msgs::StateMachinesCancel::Response &res);

    void loadEventsPlugins(const std::vector<std::string>& pluginsNames);
    void insertEvent(const std::string& event);

    void setStateMachineData(bool newState);

    ros::NodeHandlePtr _nh;

    std::shared_ptr<StateMachinesStorage> _stateMachineStorage;
    std::shared_ptr<StateMachinesBase> _stateMachineService;
    std::vector<std::shared_ptr<ReactiveInputsBase>> _reactiveInputs;

    std::shared_ptr<ReactiveBuffer> _stateMachineBuffer;
    std::shared_ptr<ReactiveBufferStorage> _reactiveBufferStorage;

    std::vector<std::string> _reactiveBuffersNames;
    std::vector<std::string> _bufferNames;

    pluginlib::ClassLoader<resource_management::EventsInterface> _loader;
    std::vector<boost::shared_ptr<resource_management::EventsInterface>> _plugins;

    ros::Subscriber _prioritiesSubscriber;
    ros::Publisher _activeBufferPublisher;
    ros::Publisher _stateMachineStatusPublisher;
    ros::ServiceServer _stateMachineCancelService;

    double _hz;
    StateMachineRunner _StateMachine;
    std::shared_ptr<StateStorage> _activeStateMachine;
    std::mutex _stateMachineMutex;
};

template<typename StateMachineType, typename ...InputDataTypes>
ResourceManager<StateMachineType,InputDataTypes...>::ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames, const std::vector<std::string>& pluginsNames, bool synchronized):
    _nh(std::move(nh)), _loader("resource_management", "resource_management::EventsInterface")
{
    loadEventsPlugins(pluginsNames);
    _stateMachineStorage = std::make_shared<StateMachinesStorage>();
    this->_stateMachineService =
            std::make_shared<StateMachines<StateMachineType>>
                                          (
                                              _nh,
                                              boost::bind(&ResourceManager<StateMachineType,InputDataTypes...>::stateFromMsg,this,_1),
                                              boost::bind(&ResourceManager<StateMachineType,InputDataTypes...>::transitionFromMsg,this,_1),
                                              boost::bind(&ResourceManager<StateMachineType,InputDataTypes...>::generateResponseMsg,this,_1),
                                              _stateMachineStorage,
                                              synchronized
                                          );
    addBufferNames(reactiveInputNames);
    createReactiveBufferStorage();
    Impl<InputDataTypes...>::add(_reactiveInputs,_nh,reactiveInputNames,*_reactiveBufferStorage);
    _activeBufferPublisher = _nh->advertise<std_msgs::String>("active_buffer", 10, true);
    _prioritiesSubscriber = _nh->subscribe("set_priorities", 10, &ResourceManager<StateMachineType,InputDataTypes...>::prioritiesCallback, this);
    _stateMachineStatusPublisher = _nh->advertise<resource_management_msgs::StateMachinesStatus>(synchronized ? "state_machine_status__" : "state_machine_status", 10);
    _stateMachineCancelService = _nh->advertiseService(synchronized ? "state_machine_cancel__" : "state_machine_cancel", &ResourceManager<StateMachineType,InputDataTypes...>::stateMachineCancel, this);

    if(!_nh->getParam("freq", _hz))
    {
      _nh->setParam("freq", 100);
      _hz = 100;
    }
}

template<typename StateMachineType, typename ...InputDataTypes>
ResourceManager<StateMachineType,InputDataTypes...>::~ResourceManager()
{
  _plugins.clear();
}

template<typename StateMachineType, typename ...InputDataTypes>
const std::vector<std::string> &ResourceManager<StateMachineType,InputDataTypes...>::getBufferNames() const
{
    return _bufferNames;
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::addBufferNames(const std::vector<std::string> &bufferNames)
{
    _reactiveBuffersNames = bufferNames;
    _bufferNames.emplace_back("state_machine");
    _bufferNames.insert(_bufferNames.end(),bufferNames.begin(),bufferNames.end());
    _bufferNames.emplace_back("artificial_life");
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::createReactiveBufferStorage()
{
    _reactiveInputs.clear();
    _reactiveBufferStorage=std::make_shared<ReactiveBufferStorage>(getBufferNames());

    _reactiveBufferStorage->setPriority("artificial_life", background);
    _artificialLifeBuffer=_reactiveBufferStorage->operator[]("artificial_life");

    _reactiveBufferStorage->setPriority("state_machine", atomic);
    _stateMachineBuffer=_reactiveBufferStorage->operator[]("state_machine");
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::prioritiesCallback(const resource_management_msgs::PrioritiesSetter& msg)
{
  size_t min = (msg.values.size() < msg.buffers.size()) ? msg.values.size() : msg.buffers.size();

  for(size_t i = 0; i < min; i++)
  {
    assert(msg.values[i] <= 4);
    assert(msg.values[i] >= -1);
    if((msg.values[i] > 4) || (msg.values[i] < -1))
    {
      ROS_ERROR_STREAM("Buffer priority out of range");
      return;
    }
  }

  focus_priority_t priority = background;
  for(size_t i = 0; i < min; i++)
  {
    switch (msg.values[i]) {
      case 4: priority = atomic; break;
      case 3: priority = prioritize; break;
      case 2: priority = normal; break;
      case 1: priority = secondary; break;
      case 0: priority = background; break;
      case -1: priority = inhibit; break;
      default: priority = background; break;
    }

    if(std::find(_reactiveBuffersNames.begin(), _reactiveBuffersNames.end(), msg.buffers[i]) != _reactiveBuffersNames.end())
      _reactiveBufferStorage->operator[](msg.buffers[i])->setPriority(priority);
  }
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::run()
{
  std::thread sm_th;
  bool state_machine_running = false;
  std::thread al_th;
  bool artificial_life_running = false;

  std::string active_buffer;

  // this in lambda is necessary for gcc <= 5.1
  _StateMachine.setPublicationFunction([this](auto state){ this->publishState(state); });
  _stateMachineStorage->setPublicationFunction([this](auto state){ this->publishState(state); });

  size_t param_update = 0;
  while (ros::ok())
  {
    _stateMachineMutex.lock();
    if(state_machine_running == false)
    {
      if(_stateMachineStorage->empty() == false)
      {
        _activeStateMachine = _stateMachineStorage->pop(_reactiveBufferStorage->getHighestPriority());
        if(_activeStateMachine)
        {
          _stateMachineStorage->setUnpoppable();
          _StateMachine.setInitialState(_activeStateMachine->getInitialState(), _activeStateMachine->getId());
          _StateMachine.setTimeout(_activeStateMachine->getTimeout());
          _StateMachine.setDeadLine(_activeStateMachine->getDeadLine());

          sm_th = std::thread(&StateMachineRunner::run, &_StateMachine);
          state_machine_running = true;
          _stateMachineMutex.unlock();
          continue;
        }
      }
    }
    else
    {
      if((_StateMachine.isWildcardState()) && (!artificial_life_running))
      {
        artificial_life_running = true;
        _artificialLife->start();
        al_th = std::thread(&ArtificialLife::run, _artificialLife);
      }
      else if((!_StateMachine.isWildcardState()) && artificial_life_running)
      {
        _artificialLife->stop();
        artificial_life_running = false;
        al_th.join();
      }

      if(!_StateMachine.runing() && state_machine_running)
        if (sm_th.joinable())
        {
          sm_th.join();
          state_machine_running = false;
          _activeStateMachine.reset();
          _stateMachineMutex.unlock();
          continue;
        }


      // for state_machine preamption by other state machine
      if(_stateMachineStorage->poppable(_reactiveBufferStorage->getHighestPriority()))
      {
        _StateMachine.addEvent("__preamted__");
        _stateMachineMutex.unlock();
        continue;
      }
    }
    setStateMachineData(_StateMachine.isNewState());
    _stateMachineMutex.unlock();

    std::shared_ptr<ReactiveBuffer> buff = _reactiveBufferStorage->getMorePriority();
    if(buff)
      if(buff->operator()())
      {
        if((buff->getName() != active_buffer) || (buff->hasItBeenPublished() == false))
          buff->operator()()->publish(true);
        else
          buff->operator()()->publish(false);
        buff->published();

        if(buff->getName() != active_buffer)
        {
          active_buffer = buff->getName();

          if((active_buffer != "state_machine") && (state_machine_running))
            _StateMachine.addEvent("__preamted__");
          if((active_buffer!= "artificial_life") && (artificial_life_running))
          {
            _artificialLife->stop();
            artificial_life_running = false;
            al_th.join();
          }
          if(active_buffer == "artificial_life")
          {
            if(!artificial_life_running)
            {
              artificial_life_running = true;
              _artificialLife->start();
              al_th = std::thread(&ArtificialLife::run, _artificialLife);
            }
          }

          std_msgs::String active_buffer_msg;
          active_buffer_msg.data = active_buffer;
          _activeBufferPublisher.publish(active_buffer_msg);
        }
      }

    if(++param_update > 10)
    {
      _nh->getParam("freq", _hz);
      param_update = 0;

      // Remove state machines that will not be executed as soon
      // as possible at low frequency
      _stateMachineStorage->clean();
    }

    ros::Rate r(_hz);
    r.sleep();
  }

  if(artificial_life_running)
  {
    _artificialLife->stop();
    al_th.join();
  }

  if(state_machine_running)
  {
    _StateMachine.addEvent("__preamted__");
    sm_th.join();
  }
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::loadEventsPlugins(const std::vector<std::string>& pluginsNames)
{
  std::vector<std::string> reasoners = _loader.getDeclaredClasses();

  for(auto name : pluginsNames)
  {
    try
    {
      _plugins.push_back(_loader.createInstance(name));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }

  for(auto plugin : _plugins)
  {
    plugin->setNodeHandle(_nh);
    plugin->registerSpreading([this](auto event){ this->insertEvent(event); });
  }
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::done()
{
  insertEvent("__done__");
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::insertEvent(const std::string& event)
{
  _StateMachine.addEvent(event);
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::setStateMachineData(bool newState)
{
  if(_activeStateMachine)
  {
    if(_StateMachine.isWildcardState())
    {
      if(_artificialLifeBuffer->hasItBeenPublished() == false)
      {
        std::shared_ptr<MessageAbstraction> tmp = _artificialLifeBuffer->getData()->clone();
        tmp->setPriority(_activeStateMachine->getStateData(_StateMachine.getCurrentStateName())->getPriority() );
        _stateMachineBuffer->setData(tmp);
        _artificialLifeBuffer->published();
      }
    }
    else if(newState)
      _stateMachineBuffer->setData(_activeStateMachine->getStateData(_StateMachine.getCurrentStateName()) );
  }
  else
    _stateMachineBuffer->setData(nullptr);
}

template<typename StateMachineType, typename ...InputDataTypes>
void ResourceManager<StateMachineType,InputDataTypes...>::publishState(StateMachineInternalState_t state)
{
  std::string state_event;
  switch (state.transition_state_) {
    case transition_pass_on_event : state_event = "pass_on_event"; break;
    case transition_pass_on_duration : state_event = "pass_on_duration"; break;
    case transition_timeout : state_event = "timeout"; break;
    case transition_wait : state_event = "wait"; break;
    case transition_global_timeout : state_event = "global_timeout"; break;
    case transition_preampt : state_event = "preampt"; break;
    case transition_dead_line : state_event = "dead_line"; break;
    case transition_none : state_event = "none"; break;
  }

  resource_management_msgs::StateMachinesStatus status;
  status.state_event = state_event;
  if(state.state_ != nullptr)
    status.state_name = state.state_->getName();
  else
    status.state_name = "";
  status.id = state.state_machine_id;

  _stateMachineStatusPublisher.publish(status);
}

template<typename StateMachineType, typename ...InputDataTypes>
bool ResourceManager<StateMachineType,InputDataTypes...>::stateMachineCancel
                          (resource_management_msgs::StateMachinesCancel::Request  &req,
                          resource_management_msgs::StateMachinesCancel::Response &res)
{
  bool found = false;

  _stateMachineMutex.lock();
  if(!_stateMachineStorage->remove(req.id))
  {
    if(_activeStateMachine)
      if(_activeStateMachine->getId() == req.id)
      {
        _StateMachine.addEvent("__preamted__");
        found = true;
      }
  }
  else
    found = true;
  _stateMachineMutex.unlock();

  res.ack = found;

  return true;
}

} // namespace resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
