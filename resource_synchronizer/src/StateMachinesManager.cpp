#include "resource_synchronizer/StateMachinesManager.h"

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE  "\x1B[1;33m"
#endif

namespace resource_synchronizer
{

void StateMachinesManager::registerHolder(StateMachinesHolderBase* holder)
{
  state_machines_holders_.push_back(holder);
}

void StateMachinesManager::insert(int id, resource_synchronizer_msgs::MetaStateMachineHeader header)
{
  headers_[id] = header;

  if(header.timeout == ros::Duration(0))
    std::cout << COLOR_ORANGE << "[WARNING] meta state machine " << id << ": timeout set to 0" << COLOR_OFF << std::endl;
}

void StateMachinesManager::cancel(int meta_sm_id)
{
  for(size_t i = 0; i < state_machines_holders_.size(); i++)
    state_machines_holders_[i]->cancel(meta_sm_id);

  sm_start_time_.erase(meta_sm_id);
  headers_.erase(meta_sm_id);
  sm_start_time_.erase(meta_sm_id);
}

void StateMachinesManager::run()
{
  ros::Rate r(100);
  run_ = true;

  done_.resize(state_machines_holders_.size());
  ids_.resize(state_machines_holders_.size());
  preempt_.resize(state_machines_holders_.size());

  while(run_ && ros::ok())
  {
    mutex_.lock();
    applyConstraints();
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
            selectSm(st_id);
          else // remove state machine not executable for this run
            removeSm(st_id);
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
        {
          state_machines_holders_[i]->send(ids_[i][0]);
          sm_start_time_[ids_[i][0]] = ros::Time::now();
          if(std::find(running_ids_.begin(), running_ids_.end(), ids_[i][0]) == running_ids_.end())
            running_ids_.push_back(ids_[i][0]);
        }
        else
        {
          cancel(running_id);
          publishPreemptStatus(running_id);

          state_machines_holders_[i]->send(ids_[i][0]);
          sm_start_time_[ids_[i][0]] = ros::Time::now();
          if(std::find(running_ids_.begin(), running_ids_.end(), ids_[i][0]) == running_ids_.end())
            running_ids_.push_back(ids_[i][0]);
        }
      }
    }
    mutex_.unlock();
    r.sleep();
  }
}

void StateMachinesManager::stop()
{
  run_ = false;
}

// PRIVATE

void StateMachinesManager::init()
{
  for(size_t i = 0; i < state_machines_holders_.size(); i++)
  {
    ids_[i] = state_machines_holders_[i]->getIdsPerPriorities();
    done_[i] = false;
    preempt_[i] = false;
  }
}

void StateMachinesManager::reinit()
{
  for(size_t i = 0; i < state_machines_holders_.size(); i++)
  {
    ids_[i] = state_machines_holders_[i]->getIdsPerPriorities();
    done_[i] = false;
  }
  cleanPreempted();
}

bool StateMachinesManager::isDone()
{
  bool res = true;
  for(const auto& done : done_)
    res = res && done;
  return res;
}

void StateMachinesManager::applyConstraints()
{
  for(const auto& it : headers_)
  {
    // is state machine is allready running
    if(std::find(running_ids_.begin(), running_ids_.end(), it.first) != running_ids_.end())
    {
      if(it.second.timeout != ros::Duration(-1))
      {
        auto st_it = sm_start_time_.find(it.first);
        if(st_it != sm_start_time_.end())
        {
          if(ros::Time::now() - st_it->second >= it.second.timeout)
          {
            publishTimeoutStatus(it.first);
            cancel(it.first);
          }
        }
      }
    }
    // if begin dead line is over
    else if((it.second.begin_dead_line != ros::Time(0)) && (it.second.begin_dead_line <= ros::Time::now()))
    {
      publishBDLStatus(it.first);
      cancel(it.first);
    }
  }
}

void StateMachinesManager::clean()
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

void StateMachinesManager::cleanPreempted()
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

void StateMachinesManager::selectSm(int id)
{
  for(size_t j = 0; j < ids_.size(); j++)
  {
    auto it = std::find(ids_[j].begin(), ids_[j].end(), id);
    if (it != ids_[j].end())
      done_[j] = true;
  }

  clean();
}

void StateMachinesManager::removeSm(int id)
{
  for(size_t j = 0; j < ids_.size(); j++)
  {
    auto it = std::find(ids_[j].begin(), ids_[j].end(), id);
    if (it != ids_[j].end())
      ids_[j].erase(it);
  }
}

bool StateMachinesManager::isSelectable(size_t owner, int id)
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

void StateMachinesManager::publishTimeoutStatus(int id)
{
  SubStateMachineStatus sub_status;
  sub_status.id = id;
  sub_status.resource = "_";
  sub_status.state_name = "";
  sub_status.event_name = "global_timeout";
  if(status_callback_)
    status_callback_(sub_status);
}

void StateMachinesManager::publishBDLStatus(int id)
{
  SubStateMachineStatus sub_status;
  sub_status.id = id;
  sub_status.resource = "_";
  sub_status.state_name = "";
  sub_status.event_name = "dead_line";
  if(status_callback_)
    status_callback_(sub_status);
}

void StateMachinesManager::publishPreemptStatus(int id)
{
  SubStateMachineStatus sub_status;
  sub_status.id = id;
  sub_status.resource = "_";
  sub_status.state_name = "";
  sub_status.event_name = "preampt";
  if(status_callback_)
    status_callback_(sub_status);
}

} //namespace resource_synchronizer
