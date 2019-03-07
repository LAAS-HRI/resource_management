#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_

#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>

#include <message_storage/ReactiveBuffer.h>
#include "message_storage/ReactiveBufferStorage.h"
#include "message_storage/MessageWrapper.h"

class ReactiveInputsBase{
};

template<class T>
class ReactiveInputs : public ReactiveInputsBase
{

public:
    ReactiveInputs(ros::NodeHandlePtr nh, const std::vector<std::string> &prio_buffer_names, const ReactiveBufferStorage &bufferStorage);

private:
    void _subscriberCallback(size_t index, const boost::shared_ptr<const T> &msg);

    ros::NodeHandlePtr _nh;
    std::vector<ros::Subscriber> _subscribers;
    std::vector<std::shared_ptr<ReactiveBuffer>> _buffers;
};

template<class T>
ReactiveInputs<T>::ReactiveInputs(ros::NodeHandlePtr nh, const std::vector<std::string> &prio_buffer_names, const ReactiveBufferStorage &bufferStorage):
    _nh(std::move(nh))
{
    for(size_t index = 0 ; index < prio_buffer_names.size(); ++index){
        _buffers.push_back(bufferStorage[prio_buffer_names[index]]);
        auto sub=_nh->subscribe<T>(prio_buffer_names[index]+ros::message_traits::md5sum<T>(),1,boost::bind(&ReactiveInputs<T>::_subscriberCallback,this,index,_1));
        _subscribers.push_back(sub);
    }
}

template<class T>
void ReactiveInputs<T>::_subscriberCallback(size_t index, const boost::shared_ptr<T const> &msg)
{
    auto wrap = std::make_shared<MessageWrapper<typename T::_data_type>>(msg->data);
    wrap->setPriority(static_cast<importance_priority_t>(msg->priority.value));
    _buffers[index]->setData(wrap);
}

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_
