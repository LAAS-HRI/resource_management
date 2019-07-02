#ifndef RESOURCEMANAGEMENT_STATESMSG_H
#define RESOURCEMANAGEMENT_STATESMSG_H

#include <string>
#include <map>
#include <vector>
#include <iostream>

#include <ros/ros.h>

#include "resource_management/API/StateMsg.h"

namespace resource_management
{

template<typename T>
class StatesMsg
{
  typedef typename T::_data_type DataType;
public:
  void addState(const std::string& name, DataType data)
  {
    states_.insert(std::pair<std::string, StateMsg<T>>(name, StateMsg<T>(name, data)));
  }

  StateMsg<T>* operator[](std::string name)
  {
    if(states_.find(name) != states_.end())
      return &states_.at(name);
    else
    {
      std::cout << "[ERROR][resource_management] Try to get unknow state" << std::endl;
      return nullptr;
    }
  }

  std::vector<T> operator()()
  {
    std::vector<T> res;
    for(auto& state : states_)
      res.push_back(state.second());
    return res;
  }
private:
  std::map<std::string, StateMsg<T>> states_;
};

} // namespace resource_management

#endif // RESOURCEMANAGEMENT_STATESMSG_H
