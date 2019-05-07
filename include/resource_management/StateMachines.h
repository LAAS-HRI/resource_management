#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATE_MACHINES_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATE_MACHINES_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <map>
#include <tuple>
#include <utility>

#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"
#include "state_machine/StateMachinesStorage.h"

#include "message_storage/MessageWrapper.h"
#include "resource_management_msgs/StateMachineTransition.h"

namespace resource_management {

class StateMachinesBase
{
};

template<class T>
class StateMachines : public StateMachinesBase
{
public:
    using StateFromMsgFn = boost::function<std::map<std::string,std::shared_ptr<MessageAbstraction>>(const typename T::Request&)>;
    using TransitionFromMsgFn = boost::function<std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>(const typename T::Request&)>;
    using GenerateResponseMsgFn = boost::function< typename T::Response(uint32_t)>;

    StateMachines(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, GenerateResponseMsgFn, std::shared_ptr<StateMachinesStorage> storage);

private:
    bool _serviceCallback(typename T::Request &req, typename T::Response &res);
    ros::NodeHandlePtr _nh;
    ros::ServiceServer _serviceServer;
    StateFromMsgFn _getStateDataFromStateMachineMsg;
    TransitionFromMsgFn _getTransitionsFromStateMachineMsg;
    GenerateResponseMsgFn _generateResponseMsg;
    std::shared_ptr<StateMachinesStorage> _storage;
    uint32_t _stateMachinesId;
};

template<class T>
StateMachines<T>::StateMachines(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, GenerateResponseMsgFn generateResponseMsg, std::shared_ptr<StateMachinesStorage> storage):
    _nh(std::move(nh)),
    _getStateDataFromStateMachineMsg(std::move(stateFromMsg)),
    _getTransitionsFromStateMachineMsg(std::move(transitionFromMsg)),
    _generateResponseMsg(std::move(generateResponseMsg))
{
    _storage = storage;
    _stateMachinesId = 0;
    _serviceServer = _nh->advertiseService("state_machines_register",&StateMachines<T>::_serviceCallback,this);
}

template<class T>
bool StateMachines<T>::_serviceCallback(typename T::Request &req, typename T::Response &res)
{
    std::shared_ptr<StateStorage> states = std::make_shared<StateStorage>(_stateMachinesId, req.header.timeout, req.header.begin_dead_line);
    states->setInitialState(req.header.initial_state);

    assert(req.header.priority.value <= 4);
    assert(req.header.priority.value >= -1);
    if((req.header.priority.value > 4) || (req.header.priority.value < -1))
    {
      ROS_ERROR_STREAM("State machine priority out of range");
      return false;
    }

    importance_priority_t priority = void_msg;
    switch (req.header.priority.value) {
      case 4: priority = vital; break;
      case 3: priority = urgent; break;
      case 2: priority = high; break;
      case 1: priority = standard; break;
      case 0: priority = low; break;
      case -1: priority = void_msg; break;
      default: priority = void_msg; break;
    }
    states->setPriority(priority);

    auto transitions = _getTransitionsFromStateMachineMsg(req);

    for(auto &t : transitions){
        resource_management_msgs::EndCondition &end_condition = std::get<2>(t);
        CoordinationTransition transition(end_condition.duration,end_condition.timeout,end_condition.regex_end_condition);
        states->addTransition(std::get<0>(t),std::get<1>(t),transition);
    }

    auto stateData = _getStateDataFromStateMachineMsg(req);
    for(auto it : stateData)
      states->addData(it.first, it.second);

    if(_storage)
    {
      _storage->push(states);

      res = _generateResponseMsg(_stateMachinesId);
      _stateMachinesId++;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("No valid state machines container");
      return false;
    }
}

} // namespace resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_STATE_MACHINES_H_
