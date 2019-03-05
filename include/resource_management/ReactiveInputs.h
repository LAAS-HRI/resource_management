#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_HPP_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_HPP_

#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>

#include <message_storage/ReactiveBuffer.h>
#include "message_storage/MessageWrapper.h"

class ReactiveInputsBase{
};

template<class T>
class ReactiveInputs : public ReactiveInputsBase
{

public:
    ReactiveInputs(ros::NodeHandlePtr nh, const std::vector<std::string> &prio_buffer_names);

private:
    void _subscriberCallback(size_t index, const T &msg);

    ros::NodeHandlePtr _nh;
    std::vector<ros::Subscriber> _subscribers;
    std::vector<ReactiveBuffer*> _buffers;
};

template<class T>
ReactiveInputs<T>::ReactiveInputs(ros::NodeHandlePtr nh, const std::vector<std::string> &prio_buffer_names):
    _nh(std::move(nh))
{
    for(size_t index = 0 ; index < prio_buffer_names.size(); ++index){
        _subscribers.emplace_back(_nh->subscribe(prio_buffer_names[index],1,std::bind(&ReactiveInputs<T>::_subscriberCallback,this,index,std::placeholders::_1)));
    }
}

template<class T>
void ReactiveInputs<T>::_subscriberCallback(size_t index, const T &msg)
{
    _buffers[index]->setData(new MessageWrapper<T>(msg));
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_HPP_
