#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_

#include <ros/ros.h>
#include <vector>
#include <map>
#include <tuple>
#include <utility>

#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"
#include "state_machine/CoordinationSignalsStorage.h"

#include "message_storage/MessageWrapper.h"
#include "resource_management/CoordinationTransition.h"

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
    using TransitionFromMsgFn = boost::function<std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>(const typename T::Request&)>;
    using GenerateResponseMsgFn = boost::function< typename T::Response(uint32_t)>;
    using HeaderFromMsgFn = boost::function<CoordinationHeader_t(const typename T::Request&)>;

    CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, HeaderFromMsgFn headerFromMsg, GenerateResponseMsgFn, std::shared_ptr<CoordinationSignalsStorage> storage);

private:
    bool _serviceCallback(typename T::Request &req, typename T::Response &res);
    ros::NodeHandlePtr _nh;
    ros::ServiceServer _serviceServer;
    StateFromMsgFn _getStateDataFromCoordinationSignalMsg;
    TransitionFromMsgFn _getTransitionsFromCoordinationSignalMsg;
    HeaderFromMsgFn _getHeaderFromCoordinationSignalMsg;
    GenerateResponseMsgFn _generateResponseMsg;
    std::shared_ptr<CoordinationSignalsStorage> _storage;
    uint32_t _coordinationSignalsId;
};

template<class T>
CoordinationSignals<T>::CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, HeaderFromMsgFn headerFromMsg, GenerateResponseMsgFn generateResponseMsg, std::shared_ptr<CoordinationSignalsStorage> storage):
    _nh(std::move(nh)),
    _getStateDataFromCoordinationSignalMsg(std::move(stateFromMsg)),
    _getTransitionsFromCoordinationSignalMsg(std::move(transitionFromMsg)),
    _getHeaderFromCoordinationSignalMsg(std::move(headerFromMsg)),
    _generateResponseMsg(std::move(generateResponseMsg))
{
    _storage = storage;
    _coordinationSignalsId = 0;
    _serviceServer = _nh->advertiseService("coordination_signals_register",&CoordinationSignals<T>::_serviceCallback,this);
}

template<class T>
bool CoordinationSignals<T>::_serviceCallback(typename T::Request &req, typename T::Response &res)
{
    CoordinationHeader_t header = _getHeaderFromCoordinationSignalMsg(req);
    std::shared_ptr<StateStorage> states = std::make_shared<StateStorage>(_coordinationSignalsId, header.time_out, header.begin_dead_line);

    importance_priority_t priority = avoid;
    switch (header.priority) {
      case 4: priority = vital; break;
      case 3: priority = urgent; break;
      case 2: priority = important; break;
      case 1: priority = helpful; break;
      case 0: priority = weak; break;
      case -1: priority = useless; break;
      default: priority = avoid; break;
    }
    states->setPriority(priority);

    auto transitions = _getTransitionsFromCoordinationSignalMsg(req);

    for(auto &t : transitions){
        resource_management::EndCondition &end_condition = std::get<2>(t);
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

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
