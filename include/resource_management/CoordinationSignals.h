#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_

#include <ros/ros.h>
#include <vector>
#include <map>
#include <tuple>
#include <utility>

#include "state_machine/CoordinationStateMachine.h"
#include "state_machine/StateStorage.h"

#include "message_storage/MessageWrapper.h"
#include "resource_management/CoordinationTransition.h"

class CoordinationSignalsBase
{
};

template<class T>
class CoordinationSignals : public CoordinationSignalsBase
{
public:
    using StateFromMsgFn = boost::function<std::map<std::string,std::shared_ptr<MessageAbstraction>>(const T&)>;
    using TransitionFromMsgFn = boost::function<std::vector<std::tuple<std::string,std::string,resource_management::EndCondition>>(const T&)>;

    CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg);

private:
    void _subscriberCallback(T msg);
    ros::NodeHandlePtr _nh;
    ros::Subscriber _subscriber;
    StateFromMsgFn _getStateDataFromCoordinationSignalMsg;
    TransitionFromMsgFn _getTransitionsFromCoordinationSignalMsg;
};

template<class T>
CoordinationSignals<T>::CoordinationSignals(ros::NodeHandlePtr nh, StateFromMsgFn stateFromMsg, TransitionFromMsgFn transitionFromMsg):
    _nh(std::move(nh)), _getStateDataFromCoordinationSignalMsg(std::move(stateFromMsg)), _getTransitionsFromCoordinationSignalMsg(std::move(transitionFromMsg))
{
    _subscriber = _nh->subscribe<T>("coordination_signals",100,&CoordinationSignals<T>::_subscriberCallback,this);
}

template<class T>
void CoordinationSignals<T>::_subscriberCallback(T msg)
{
    StateStorage states; // FIXME: this should be unique/global?

    auto stateData = _getStateDataFromCoordinationSignalMsg(msg);
    // TODO: store coordination signal state data
    auto transitions = _getTransitionsFromCoordinationSignalMsg(msg);

    for(auto &t : transitions){
        resource_management::EndCondition &end_condition = std::get<2>(t);
        CoordinationTransition transition(end_condition.duration,end_condition.timeout,end_condition.regex_end_condition);
        states.addTransition(std::get<0>(t),std::get<1>(t),transition);
    }

}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_COORDINATION_SIGNALS_H_
