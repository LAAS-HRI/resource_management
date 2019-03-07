#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_

#include <vector>
#include <string>
#include <utility>

#include <ros/ros.h>

#include "resource_management/ReactiveInputs.h"
#include "resource_management/CoordinationSignals.h"
#include "resource_management/PrioritiesSetter.h"

#include "message_storage/ReactiveBuffer.h"


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
private:

    const std::vector<std::string> &getBufferNames() const;
    void addBufferNames(const std::vector<std::string> &bufferNames);

    /// initializes buffer storage
    /// also clear _reactiveInputs and _reactiveBuffers
    void createReactiveBufferStorage();

    void prioritiesCallback(const resource_management::PrioritiesSetter& msg);

    ros::NodeHandlePtr _nh;

    std::shared_ptr<CoordinationSignalsBase> _coordinationSignalService;
    std::vector<std::shared_ptr<ReactiveInputsBase>> _reactiveInputs;

    std::shared_ptr<ReactiveBuffer> _artificialLifeBuffer;
    std::shared_ptr<ReactiveBuffer> _coordinationSignalBuffer;
    std::shared_ptr<ReactiveBufferStorage> _reactiveBufferStorage;

    std::vector<std::string> _reactiveBuffersNames;
    std::vector<std::string> _bufferNames;

    ros::Subscriber _prioritiesSubscriber;
    double hz_;
};


template<typename CoordinationSignalType, typename ...InputDataTypes>
ResourceManager<CoordinationSignalType,InputDataTypes...>::ResourceManager(ros::NodeHandlePtr nh, std::vector<std::string> reactiveInputNames):
    _nh(std::move(nh)), _bufferNames({"artificial_life","coordination_signals"})
{
    this->_coordinationSignalService =
            std::make_shared<CoordinationSignals<CoordinationSignalType>>
                                          (
                                              _nh,
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::stateFromMsg,this,_1),
                                              boost::bind(&ResourceManager<CoordinationSignalType,InputDataTypes...>::transitionFromMsg,this,_1)
                                          );
    createReactiveBufferStorage();
    addBufferNames(reactiveInputNames);
    Impl<InputDataTypes...>::add(_reactiveInputs,_nh,reactiveInputNames,*_reactiveBufferStorage);
    _prioritiesSubscriber = _nh->subscribe("set_priorities", 10, &ResourceManager<CoordinationSignalType,InputDataTypes...>::prioritiesCallback, this);

    if(!_nh->getParam("freq", hz_))
    {
      _nh->setParam("freq", 100);
      hz_ = 100;
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
  size_t param_update = 0;
  while (ros::ok())
  {
    std::shared_ptr<ReactiveBuffer> buff = _reactiveBufferStorage->getMorePriority();
    if(buff)
      if(buff->operator()())
      {
        buff->operator()()->publish();
      }

    if(++param_update > 10)
    {
      _nh->getParam("freq", hz_);
      param_update = 0;
    }

    ros::Rate r(hz_);
    r.sleep();
  }
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_RESOURCE_MANAGER_H_
