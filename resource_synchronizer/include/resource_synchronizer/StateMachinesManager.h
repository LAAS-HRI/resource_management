#ifndef RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
#define RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H

#include <vector>
#include <atomic>

#include <ros/ros.h>

#include "resource_synchronizer/StateMachinesHolder.h"

namespace resource_synchronizer
{

class StateMachinesManager
{
public:
  StateMachinesManager() : run_(false) {}
  ~StateMachinesManager() {}

  void registerHolder(StateMachinesHolderBase* holder)
  {
    state_machines_holders_.push_back(holder);
  }

  void run()
  {
    ros::Rate r(100);
    run_ = true;

    done_.resize(state_machines_holders_.size());
    ids_.resize(state_machines_holders_.size());

    while(run_ && ros::ok())
    {
      init();

      while(!isDone())
      {
        for(size_t i = 0; i < ids_.size(); i++)
        {
          int st_id = -1;
          if(ids_[i].size() && !done_[i])
            st_id = ids_[i][0];
          else
            done_[i] = true;

          if(st_id != -1)
          {
            bool to_take = true;
            for(size_t j = 0; j < ids_.size(); j++)
            {
              if(i != j)
              {
                auto it = std::find(ids_[j].begin(), ids_[j].end(), st_id);
                if (it != ids_[j].end() && it != ids_[j].begin())
                  to_take = false;
              }
            }

            if(to_take)
            {
              for(size_t j = 0; j < ids_.size(); j++)
              {
                auto it = std::find(ids_[j].begin(), ids_[j].end(), st_id);
                if (it != ids_[j].end())
                  done_[j] = true;
              }

              clean();
            }
          }
        }
      }

      r.sleep();
    }
  }

  void stop()
  {
    run_ = false;
  }

private:
  std::vector<StateMachinesHolderBase*> state_machines_holders_;
  std::atomic<bool> run_;

  std::vector<bool> done_;
  std::vector<std::vector<int> > ids_;

  void init()
  {
    for(size_t i = 0; i < state_machines_holders_.size(); i++)
    {
      ids_[i] = state_machines_holders_[i]->getIdsPerPriorities();
      done_[i] = false;
    }
  }

  bool isDone()
  {
    bool res = true;
    for(const auto& done : done_)
      res = res && done;
    return res;
  }

  void clean()
  {
    for(size_t i = 0; i < ids_.size(); i++)
    {
      if(done_[i] && (ids_[i].size() > 1))
      {
        for(size_t j = 1; j < ids_[i].size();)
        {
          for(size_t k = 0; k < ids_.size(); k++)
          {
            if(i != k)
            {
              auto it = std::find(ids_[k].begin(), ids_[k].end(), ids_[i][j]);
              if(it != ids_[k].end())
                ids_[k].erase(it);
            }
          }
          ids_[i].erase(ids_[i].begin() + j);
        }
      }
    }
  }
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
