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

class CoordinationSignalsBase
{
};

template<class T>
class CoordinationSignals : public CoordinationSignalsBase
{
public:
    using StateFromMsgFn = boost::function<std::map<std::string,std::shared_ptr<MessageAbstraction>>(const typename T::Request&)>;
    using TransitionFromMsgFn = boost::function<std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>(const typename T::Request&)>;

    CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, std::shared_ptr<CoordinationSignalsStorage> storage);

private:
    bool _serviceCallback(typename T::Request &req, typename T::Response &res);
    ros::NodeHandlePtr _nh;
    ros::ServiceServer _serviceServer;
    StateFromMsgFn _getStateDataFromCoordinationSignalMsg;
    TransitionFromMsgFn _getTransitionsFromCoordinationSignalMsg;
    std::shared_ptr<CoordinationSignalsStorage> _storage;
};

template<class T>
CoordinationSignals<T>::CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg, std::shared_ptr<CoordinationSignalsStorage> storage):
    _nh(std::move(nh)), _getStateDataFromCoordinationSignalMsg(std::move(stateFromMsg)), _getTransitionsFromCoordinationSignalMsg(std::move(transitionFromMsg))
{
    _storage = storage;
    _serviceServer = _nh->advertiseService("coordination_signals",&CoordinationSignals<T>::_serviceCallback,this);
}

template<class T>
bool CoordinationSignals<T>::_serviceCallback(typename T::Request &req, typename T::Response &res)
{
    std::shared_ptr<StateStorage> states = std::make_shared<StateStorage>();

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
      return true;
    }
    else
      return false;
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
