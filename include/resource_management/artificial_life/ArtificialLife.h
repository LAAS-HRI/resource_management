#ifndef ARTIFICIALLIFE_H
#define ARTIFICIALLIFE_H

#include "ros/ros.h"

#include "resource_management/message_storage/ReactiveBuffer.h"

namespace resource_management {

class ArtificialLife
{
public:
  ArtificialLife(float frame_rate, std::shared_ptr<ReactiveBuffer> buffer) : _rate(frame_rate)
  {
    _buffer = buffer;
    _is_running = false;
  }

  void start()
  {
    _is_running = true;
    init();
  }

  void stop()
  {
    _is_running = false;
  }

  void run()
  {
    while(_is_running && ros::ok())
    {
      inLoop();
      _rate.sleep();
    }
  }

protected:
  virtual void inLoop() = 0;
  virtual void init() = 0;

  std::shared_ptr<ReactiveBuffer> _buffer;

private:
  ros::Rate _rate;
  bool _is_running;
};

} // namespace resource_management

#endif // ARTIFICIALLIFE_H
