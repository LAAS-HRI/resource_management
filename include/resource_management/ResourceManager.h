#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_

#include <vector>
#include <string>
#include <utility>
#include <thread>

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "resource_management/ReactiveInputs.h"
#include "resource_management/CoordinationSignals.h"
#include "resource_management/PrioritiesSetter.h"

#include "message_storage/ReactiveBuffer.h"

#include "artificial_life/ArtificialLife.h"

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
    ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames);

    void run();

protected:
    virtual std::map<std::string,std::shared_ptr<MessageAbstraction>> stateFromMsg(const CoordinationSignalType &msg) = 0;
    virtual std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>
        transitionFromMsg(const CoordinationSignalType &msg) = 0;

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

    ros::NodeHandlePtr _nh;

    std::shared_ptr<CoordinationSignalsStorage> _coordinationSignalStorage;
    std::shared_ptr<CoordinationSignalsBase> _coordinationSignalService;
    std::vector<std::shared_ptr<ReactiveInputsBase>> _reactiveInputs;

    std::shared_ptr<ReactiveBuffer> _coordinationSignalBuffer;
    std::shared_ptr<ReactiveBufferStorage> _reactiveBufferStorage;

    std::vector<std::string> _reactiveBuffersNames;
    std::vector<std::string> _bufferNames;

    ros::Subscriber _prioritiesSubscriber;
    ros::Publisher _activeBufferPublisher;
    double _hz;
    std::string _active_buffer;
    CoordinationStateMachine _StateMachine;
};


template<typename CoordinationSignalType, typename ...InputDataTypes>
ResourceManager<CoordinationSignalType,InputDataTypes...>::ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames):
    _nh(std::move(nh)), _bufferNames({"artificial_life","coordination_signals"})
{
    _coordinationSignalStorage = std::make_shared<CoordinationSignalsStorage>();
    this->_coordinationSignalService =
            std::make_shared<CoordinationSignals<CoordinationSignalType>>
                                          (
                                              _nh,
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::stateFromMsg,this,_1),
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::transitionFromMsg,this,_1),
                                              _coordinationSignalStorage
                                          );
    addBufferNames(reactiveInputNames);
    createReactiveBufferStorage();
    Impl<InputDataTypes...>::add(_reactiveInputs,_nh,reactiveInputNames,*_reactiveBufferStorage);
    _activeBufferPublisher = _nh->advertise<std_msgs::String>("active_buffer", 10, true);
    _prioritiesSubscriber = _nh->subscribe("set_priorities", 10, &ResourceManager<CoordinationSignalType,InputDataTypes...>::prioritiesCallback, this);

    _active_buffer = "";
    if(!_nh->getParam("freq", _hz))
    {
      _nh->setParam("freq", 100);
      _hz = 100;
    }
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
    _reactiveBuffersNames.clear();
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
  std::shared_ptr<StateStorage> current_state;
  std::thread sm_th;
  bool coordination_running = false;
  std::thread al_th;
  bool artificial_life_running = false;

  _StateMachine.setPublicationFunction([this](auto state){ publishState(state); });

  size_t param_update = 0;
  while (ros::ok())
  {
    if(coordination_running == false)
    {
      if(_coordinationSignalStorage->empty() == false)
      {
        current_state = _coordinationSignalStorage->pop();
        _StateMachine.setInitialState(current_state->getInitialState());
        _StateMachine.setTimeout(current_state->getTimeout());
        _StateMachine.setDeadLine(current_state->getDeadLine());

        sm_th = std::thread(&CoordinationStateMachine::run, &_StateMachine);
        coordination_running = true;
        continue;
      }
    }
    else
    {
      if (sm_th.joinable())
      {
        sm_th.join();
        coordination_running = false;
        continue;
      }
    }

    std::shared_ptr<ReactiveBuffer> buff = _reactiveBufferStorage->getMorePriority();
    if(buff)
      if(buff->operator()())
      {
        buff->operator()()->publish();
        if(buff->getName() != _active_buffer)
        {
          _active_buffer = buff->getName();

          if((_active_buffer != "coordination_signals") && (coordination_running))
            _StateMachine.addEvent("__preamted__");
          if((_active_buffer!= "artificial_life") && (artificial_life_running))
            _artificialLife->stop();
          if(_active_buffer == "artificial_life")
          {
            if(!artificial_life_running)
            {
              artificial_life_running = true;
              _artificialLife->start();
              al_th = std::thread(&ArtificialLife::run, _artificialLife);
            }
          }

          std_msgs::String active_buffer_msg;
          active_buffer_msg.data = _active_buffer;
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
}

template<typename CoordinationSignalType, typename ...InputDataTypes>
void ResourceManager<CoordinationSignalType,InputDataTypes...>::publishState(CoordinationInternalState_t state)
{
  std::chrono::time_point<std::chrono::system_clock> now_point = std::chrono::system_clock::now();
  std::time_t now = std::chrono::system_clock::to_time_t(now_point);
  std::cout << "[" << std::ctime(&now) << "] ";

  std::cout << "[STATE] ";
  if(state.state_ != nullptr)
    std::cout << state.state_->getName() << " : ";
  else
    std::cout << "end : ";

  switch (state.transition_state_) {
    case transition_pass_on_event : std::cout << "pass_on_event"; break;
    case transition_pass_on_duration : std::cout << "pass_on_duration"; break;
    case transition_timeout : std::cout << "timeout"; break;
    case transition_wait : std::cout << "wait"; break;
    case transition_global_timeout : std::cout << "global_timeout"; break;
    case transition_preampt : std::cout << "preampt"; break;
    case transition_dead_line : std::cout << "dead_line"; break;
    case transition_none : std::cout << "none"; break;
  }
  std::cout << std::endl;
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
