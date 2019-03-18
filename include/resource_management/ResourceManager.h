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
#include "resource_management/CoordinationSignals.h"
#include "resource_management/PrioritiesSetter.h"
#include "resource_management/CoordinationSignalsStatus.h"
#include "resource_management/CoordinationSignalsCancel.h"

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

template<typename CoordinationSignalType, typename ...InputDataTypes>
class ResourceManager
{
public:
    /// also creates 2 buffers for artificial life and coordination signals
    ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames, const std::vector<std::string>& pluginsNames);
    ~ResourceManager();

    void run();

protected:
    virtual std::map<std::string,std::shared_ptr<MessageAbstraction>> stateFromMsg(const typename CoordinationSignalType::Request &msg) = 0;
    virtual std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
        transitionFromMsg(const typename CoordinationSignalType::Request &msg) = 0;
    virtual typename CoordinationSignalType::Response generateResponseMsg(uint32_t id) = 0;

    std::shared_ptr<ArtificialLife> _artificialLife;
    std::shared_ptr<ReactiveBuffer> _artificialLifeBuffer;

private:

    const std::vector<std::string> &getBufferNames() const;
    void addBufferNames(const std::vector<std::string> &bufferNames);

    /// initializes buffer storage
    /// also clear _reactiveInputs and _reactiveBuffers
    void createReactiveBufferStorage();

    void prioritiesCallback(const resource_management::PrioritiesSetter& msg);
    void publishState(CoordinationInternalState_t state);
    bool coordinationSignalCancel(resource_management::CoordinationSignalsCancel::Request  &req,
                                  resource_management::CoordinationSignalsCancel::Response &res);

    void loadEventsPlugins(const std::vector<std::string>& pluginsNames);
    void insertEvent(const std::string& event);

    void setCoordinationSignalData();

    ros::NodeHandlePtr _nh;

    std::shared_ptr<CoordinationSignalsStorage> _coordinationSignalStorage;
    std::shared_ptr<CoordinationSignalsBase> _coordinationSignalService;
    std::vector<std::shared_ptr<ReactiveInputsBase>> _reactiveInputs;

    std::shared_ptr<ReactiveBuffer> _coordinationSignalBuffer;
    std::shared_ptr<ReactiveBufferStorage> _reactiveBufferStorage;

    std::vector<std::string> _reactiveBuffersNames;
    std::vector<std::string> _bufferNames;

    pluginlib::ClassLoader<resource_management::EventsInterface> _loader;
    std::vector<boost::shared_ptr<resource_management::EventsInterface>> _plugins;

    ros::Subscriber _prioritiesSubscriber;
    ros::Publisher _activeBufferPublisher;
    ros::Publisher _coordinationSignalStatusPublisher;
    ros::ServiceServer _coordinationSignalCancelService;

    double _hz;
    CoordinationStateMachine _StateMachine;
    std::shared_ptr<StateStorage> _activeCoordinationSignal;
    std::mutex _coordinationMutex;
};

template<typename CoordinationSignalType, typename ...InputDataTypes>
ResourceManager<CoordinationSignalType,InputDataTypes...>::ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames, const std::vector<std::string>& pluginsNames):
    _nh(std::move(nh)), _bufferNames({"artificial_life","coordination_signals"}), _loader("resource_management", "resource_management::EventsInterface")
{
    loadEventsPlugins(pluginsNames);
    _coordinationSignalStorage = std::make_shared<CoordinationSignalsStorage>();
    this->_coordinationSignalService =
            std::make_shared<CoordinationSignals<CoordinationSignalType>>
                                          (
                                              _nh,
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::stateFromMsg,this,_1),
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::transitionFromMsg,this,_1),
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::generateResponseMsg,this,_1),
                                              _coordinationSignalStorage
                                          );
    addBufferNames(reactiveInputNames);
    createReactiveBufferStorage();
    Impl<InputDataTypes...>::add(_reactiveInputs,_nh,reactiveInputNames,*_reactiveBufferStorage);
    _activeBufferPublisher = _nh->advertise<std_msgs::String>("active_buffer", 10, true);
    _prioritiesSubscriber = _nh->subscribe("set_priorities", 10, &ResourceManager<CoordinationSignalType,InputDataTypes...>::prioritiesCallback, this);
    _coordinationSignalStatusPublisher = _nh->advertise<resource_management::CoordinationSignalsStatus>("coordination_signal_status", 10);
    _coordinationSignalCancelService = _nh->advertiseService("coordination_signal_cancel", &ResourceManager<CoordinationSignalType,InputDataTypes...>::coordinationSignalCancel, this);

    if(!_nh->getParam("freq", _hz))
    {
      _nh->setParam("freq", 100);
      _hz = 100;
    }
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
ResourceManager<CoordinationSignalType,InputDataTypes...>::~ResourceManager()
{
  _plugins.clear();
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
const std::vector<std::string> &ResourceManager<CoordinationSignalType,InputDataTypes...>::getBufferNames() const
{
    return _bufferNames;
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::addBufferNames(const std::vector<std::string> &bufferNames)
{
    _reactiveBuffersNames.insert(_reactiveBuffersNames.end(),bufferNames.begin(),bufferNames.end());
    _bufferNames.insert(_bufferNames.end(),bufferNames.begin(),bufferNames.end());
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::createReactiveBufferStorage()
{
    _reactiveInputs.clear();
    _reactiveBufferStorage=std::make_shared<ReactiveBufferStorage>(getBufferNames());

    _reactiveBufferStorage->setPriority("artificial_life", fullfocus);
    _artificialLifeBuffer=_reactiveBufferStorage->operator[]("artificial_life");

    _reactiveBufferStorage->setPriority("coordination_signals", fullfocus);
    _coordinationSignalBuffer=_reactiveBufferStorage->operator[]("coordination_signals");
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::prioritiesCallback(const resource_management::PrioritiesSetter& msg)
{
  size_t min = (msg.values.size() < msg.buffers.size()) ? msg.values.size() : msg.buffers.size();

  for(size_t i = 0; i < min; i++)
  {
    assert(msg.values[i] <= 4);
    assert(msg.values[i] >= 0);
    if((msg.values[i] > 4) || (msg.values[i] < 0))
    {
      ROS_ERROR_STREAM("Buffer priority out of range");
      return;
    }
  }

  focus_priority_t priority = ignore;
  for(size_t i = 0; i < min; i++)
  {
    switch (msg.values[i]) {
      case 4: priority = fullfocus; break;
      case 3: priority = prioritize; break;
      case 2: priority = normal; break;
      case 1: priority = secondary; break;
      default: priority = ignore; break;
    }

    if(std::find(_reactiveBuffersNames.begin(), _reactiveBuffersNames.end(), msg.buffers[i]) != _reactiveBuffersNames.end())
      _reactiveBufferStorage->operator[](msg.buffers[i])->setPriority(priority);
  }
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::run()
{
  std::thread sm_th;
  bool coordination_running = false;
  std::thread al_th;
  bool artificial_life_running = false;

  std::string active_buffer;

  // this in lambda is necessary for gcc <= 5.1
  _StateMachine.setPublicationFunction([this](auto state){ this->publishState(state); });

  size_t param_update = 0;
  while (ros::ok())
  {
    _coordinationMutex.lock();
    if(coordination_running == false)
    {
      if(_coordinationSignalStorage->empty() == false)
      {
        _activeCoordinationSignal = _coordinationSignalStorage->pop();
        _StateMachine.setInitialState(_activeCoordinationSignal->getInitialState());
        _StateMachine.setTimeout(_activeCoordinationSignal->getTimeout());
        _StateMachine.setDeadLine(_activeCoordinationSignal->getDeadLine());

        sm_th = std::thread(&CoordinationStateMachine::run, &_StateMachine);
        coordination_running = true;
        _coordinationMutex.unlock();
        continue;
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

      if(!_StateMachine.runing())
        if (sm_th.joinable())
        {
          sm_th.join();
          coordination_running = false;
          _activeCoordinationSignal = std::make_shared<StateStorage>();
          _coordinationMutex.unlock();
          continue;
        }
    }
    setCoordinationSignalData();
    _coordinationMutex.unlock();

    std::shared_ptr<ReactiveBuffer> buff = _reactiveBufferStorage->getMorePriority();
    if(buff)
      if(buff->operator()())
      {
        buff->operator()()->publish();
        if(buff->getName() != active_buffer)
        {
          active_buffer = buff->getName();

          if((active_buffer != "coordination_signals") && (coordination_running))
          {
            std::cout << "P2 " << active_buffer << std::endl;
            _StateMachine.addEvent("__preamted__");
          }
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
    }

    ros::Rate r(_hz);
    r.sleep();
  }

  if(artificial_life_running)
  {
    _artificialLife->stop();
    al_th.join();
  }

  if(coordination_running)
  {
    _StateMachine.addEvent("__preamted__");
    sm_th.join();
  }
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::loadEventsPlugins(const std::vector<std::string>& pluginsNames)
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
    plugin->registerSpreading([this](auto event){ this->insertEvent(event); });
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::insertEvent(const std::string& event)
{
  _StateMachine.addEvent(event);
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::setCoordinationSignalData()
{
  if(_activeCoordinationSignal)
  {
    if(_StateMachine.isWildcardState())
    {
      std::shared_ptr<MessageAbstraction> tmp = _artificialLifeBuffer->getData()->clone();
      tmp->setPriority(_activeCoordinationSignal->getStateData(_StateMachine.getCurrentStateName())->getPriority() );
      _coordinationSignalBuffer->setData(tmp);
    }
    else
      _coordinationSignalBuffer->setData(_activeCoordinationSignal->getStateData(_StateMachine.getCurrentStateName()) );
  }
  else
    _coordinationSignalBuffer->setData(nullptr);
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::publishState(CoordinationInternalState_t state)
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

  resource_management::CoordinationSignalsStatus status;
  status.state_event = state_event;
  if(state.state_ != nullptr)
    status.state_name = state.state_->getName();
  else
    status.state_name = "";
  status.id = state.state_machine_id;

  _coordinationSignalStatusPublisher.publish(status);
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
bool ResourceManager<CoordinationSignalType,InputDataTypes...>::coordinationSignalCancel
                          (resource_management::CoordinationSignalsCancel::Request  &req,
                          resource_management::CoordinationSignalsCancel::Response &res)
{
  bool found = false;

  _coordinationMutex.lock();
  if(!_coordinationSignalStorage->remove(req.id))
  {
    if(_activeCoordinationSignal)
      if(_activeCoordinationSignal->getId() == req.id)
      {
        _StateMachine.addEvent("__preamted__");
        found = true;
      }
  }
  else
    found = true;
  _coordinationMutex.unlock();

  res.ack = found;

  return true;
}

} // namespace resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
