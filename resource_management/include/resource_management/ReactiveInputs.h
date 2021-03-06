#ifndef _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_
#define _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_

#include <string>
#include <vector>
#include <utility>
#include <regex>

#include <ros/ros.h>
#include <ros/console.h>

#include "resource_management/message_storage/ReactiveBuffer.h"
#include "resource_management/message_storage/ReactiveBufferStorage.h"
#include "resource_management/message_storage/MessageWrapper.h"

namespace resource_management {

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
    std::regex regex_name("^N\\d+(.*)\\d+(.*)_ISaIvEEE$");
    std::smatch match;

    for(size_t index = 0; index < prio_buffer_names.size(); ++index)
    {
        _buffers.push_back(bufferStorage[prio_buffer_names[index]]);

        std::string sub_name = prio_buffer_names[index] + "/";
        std::string type_id = typeid(T).name();
        if(std::regex_match(type_id, match, regex_name))
          sub_name += match[1].str() + "_" + match[2].str();
        else
          sub_name += typeid(T).name();

        auto sub=_nh->subscribe<T>(sub_name,1,boost::bind(&ReactiveInputs<T>::_subscriberCallback,this,index,_1));
        _subscribers.push_back(sub);
    }
}

template<class T>
void ReactiveInputs<T>::_subscriberCallback(size_t index, const boost::shared_ptr<T const> &msg)
{
    auto wrap = std::make_shared<MessageWrapper<typename T::_data_type>>(msg->data);

    assert(msg->priority.value <= 4);
    assert(msg->priority.value >= -1);
    if((msg->priority.value > 4) || (msg->priority.value < -1))
    {
      ROS_ERROR_STREAM("Reactive message priority out of range");
      return;
    }

    importance_priority_t priority = void_msg;
    switch (msg->priority.value) {
      case 4: priority = vital; break;
      case 3: priority = urgent; break;
      case 2: priority = high; break;
      case 1: priority = standard; break;
      case 0: priority = low; break;
      case -1: priority = void_msg; break;
      default: priority = void_msg; break;
    }

    wrap->setPriority(priority);
    _buffers[index]->setData(wrap);
}

} // namespace resource_management

#endif // _RESOURCE_MANAGEMENT_INCLUDE_RESOURCE_MANAGEMENT_REACTIVE_INPUTS_H_
