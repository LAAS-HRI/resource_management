#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <map>
#include <tuple>
#include <utility>

#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"
#include "state_machine/CoordinationSignalsStorage.h"

#include "message_storage/MessageWrapper.h"
#include "resource_management_msgs/CoordinationSignalsTransition.h"

namespace resource_management {

struct CoordinationHeader_t
{
  ros::Duration time_out;
  ros::Time begin_dead_line;
  int priority;
};

class CoordinationSignalsBase
{
};

template<class T>
class CoordinationSignals : public CoordinationSignalsBase
{
public:
    using StateFromMsgFn = boost::function<std::map<std::string,std::shared_ptr<MessageAbstraction>>(const typename T::Request&)>;
    using TransitionFromMsgFn = boost::function<std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>(const typename T::Request&)>;
    using GenerateResponseMsgFn = boost::function< typename T::Response(uint32_t)>;

    CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, GenerateResponseMsgFn, std::shared_ptr<CoordinationSignalsStorage> storage);

private:
    bool _serviceCallback(typename T::Request &req, typename T::Response &res);
    ros::NodeHandlePtr _nh;
    ros::ServiceServer _serviceServer;
    StateFromMsgFn _getStateDataFromCoordinationSignalMsg;
    TransitionFromMsgFn _getTransitionsFromCoordinationSignalMsg;
    GenerateResponseMsgFn _generateResponseMsg;
    std::shared_ptr<CoordinationSignalsStorage> _storage;
    uint32_t _coordinationSignalsId;
};

template<class T>
CoordinationSignals<T>::CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, GenerateResponseMsgFn generateResponseMsg, std::shared_ptr<CoordinationSignalsStorage> storage):
    _nh(std::move(nh)),
    _getStateDataFromCoordinationSignalMsg(std::move(stateFromMsg)),
    _getTransitionsFromCoordinationSignalMsg(std::move(transitionFromMsg)),
    _generateResponseMsg(std::move(generateResponseMsg))
{
    _storage = storage;
    _coordinationSignalsId = 0;
    _serviceServer = _nh->advertiseService("coordination_signals_register",&CoordinationSignals<T>::_serviceCallback,this);
}

template<class T>
bool CoordinationSignals<T>::_serviceCallback(typename T::Request &req, typename T::Response &res)
{
    std::shared_ptr<StateStorage> states = std::make_shared<StateStorage>(_coordinationSignalsId, req.header.timeout, req.header.begin_dead_line);
    states->setInitialState(req.header.initial_state);

    assert(req.header.priority.value <= 4);
    assert(req.header.priority.value >= -1);
    if((req.header.priority.value > 4) || (req.header.priority.value < -1))
    {
      ROS_ERROR_STREAM("Coordination signal priority out of range");
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

    auto transitions = _getTransitionsFromCoordinationSignalMsg(req);

    for(auto &t : transitions){
        resource_management_msgs::EndCondition &end_condition = std::get<2>(t);
        CoordinationTransition transition(end_condition.duration,end_condition.timeout,end_condition.regex_end_condition);
        states->addTransition(std::get<0>(t),std::get<1>(t),transition);
    }

    auto stateData = _getStateDataFromCoordinationSignalMsg(req);
    for(auto it : stateData)
      states->addData(it.first, it.second);

    if(_storage)
    {
      _storage->push(states);

      res = _generateResponseMsg(_coordinationSignalsId);
      _coordinationSignalsId++;
      return true;
    }
    else
      return false;
}

} // namespace resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
