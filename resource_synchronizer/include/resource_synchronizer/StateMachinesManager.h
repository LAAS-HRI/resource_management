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
    preempt_.resize(state_machines_holders_.size());

    while(run_ && ros::ok())
    {
      init();

      while(!isDone())
      {
        for(size_t i = 0; i < ids_.size(); i++)
        {

          //get state machine to run
          int st_id = -1;
          if(ids_[i].size() && !done_[i])
            st_id = ids_[i][0];
          else
            done_[i] = true;

          if(st_id != -1) // if at least one state machine to run for machine i
          {
            bool to_take = true; // if this id can be took

            if(preempt_[i]) // don't concider the running machine because need to be preempted
            {
              to_take = isSelectable(i, st_id);
            }
            else if(state_machines_holders_[i]->canReplace(st_id)) // if machine st_id can preempt_ the one running
            {
              to_take = isSelectable(i, st_id);
              if(to_take && (state_machines_holders_[i]->isRunning() != -1))
              {
                preempt_[i] = true;
                reinit();
                continue;
                // we set this state machime preemptable but do not affect it for the momemnt
              }
            }
            else // the one running is more important
              to_take = false;

            if(to_take)
              select(st_id);
            else // remove state machine not executable for this run
              remove(st_id);
          }
        }
      }

      // ids to run are in ids_
      for(size_t i = 0; i < ids_.size(); i++)
      {
        if(ids_[i].size())
        {
          int running_id = state_machines_holders_[i]->isRunning();
          if(running_id == ids_[i][0])
            continue;
          else if(running_id == -1)
            state_machines_holders_[i]->send(ids_[i][0]);
          else
          {
            state_machines_holders_[i]->cancel();
            state_machines_holders_[i]->send(ids_[i][0]);
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
  std::vector<bool> preempt_;

  void init()
  {
    for(size_t i = 0; i < state_machines_holders_.size(); i++)
    {
      ids_[i] = state_machines_holders_[i]->getIdsPerPriorities();
      done_[i] = false;
      preempt_[i] = false;
    }
  }

  void reinit()
  {
    for(size_t i = 0; i < state_machines_holders_.size(); i++)
    {
      ids_[i] = state_machines_holders_[i]->getIdsPerPriorities();
      done_[i] = false;
    }
    cleanPreempted();
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

  void cleanPreempted()
  {
    for(size_t i = 0; i < ids_.size(); i++)
    {
      if(preempt_[i])
      {
        int preempt_id = state_machines_holders_[i]->isRunning();
        if(preempt_id != -1)
        {
          for(size_t j = 0; j < ids_.size(); j++)
          {
            if(state_machines_holders_[j]->isOneRunning(preempt_id))
              preempt_[j] = true;

            auto it = std::find(ids_[j].begin(), ids_[j].end(), preempt_id);
            if(it != ids_[j].end())
              ids_[j].erase(it);
          }
        }
      }
    }
  }

  void select(int id)
  {
    for(size_t j = 0; j < ids_.size(); j++)
    {
      auto it = std::find(ids_[j].begin(), ids_[j].end(), id);
      if (it != ids_[j].end())
        done_[j] = true;
    }

    clean();
  }

  void remove(int id)
  {
    for(size_t j = 0; j < ids_.size(); j++)
    {
      auto it = std::find(ids_[j].begin(), ids_[j].end(), id);
      if (it != ids_[j].end())
        ids_[j].erase(it);
    }
  }

  bool isSelectable(size_t owner, int id)
  {
    bool res = true;

    for(size_t j = 0; j < ids_.size(); j++)
    {
      if(j != owner)
      {
        auto it = std::find(ids_[j].begin(), ids_[j].end(), id);
        if (it != ids_[j].end() && it != ids_[j].begin()) // if present but not a position 0
          res = false;
        else if((it != ids_[j].end()) && (!state_machines_holders_[j]->canReplace(id)) && !preempt_[j])
        // if at position 0 and can not preempt the running one (not preemtable and not allready preempted)
          res = false;
      }
    }

    return res;
  }
};

} // namespace resource_synchronizer

#endif // RESOURCE_SYNCHRONIZER_STATEMACHINESMANAGER_H
