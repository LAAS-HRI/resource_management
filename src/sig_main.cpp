#include "message_storage/MessageWrapper.h"
#include "message_storage/ReactiveBuffer.h"
#include "message_storage/ReactiveBufferStorage.h"

#include <iostream>
#include <thread>
#include <unistd.h>
#include <string>

struct test_t
{
  std::string msg;
};

void publishStrMsg(std::string msg)
{
  std::cout << "[StrMsg] " << msg << std::endl;
}

void publishTestMsg(test_t msg)
{
  std::cout << "[TestMsg] "  << msg.msg << std::endl;
}

int main(int argc, char** argv)
{
  /******** MessageWrapper ******/
  MessageWrapper<std::string>::registerPublishFunction(&publishStrMsg);
  MessageWrapper<test_t>::registerPublishFunction(&publishTestMsg);

  MessageWrapper<std::string> m1("m1");
  MessageWrapper<std::string> m2;
  m2 = "m2";

  m1.publish();
  m2.publish();

  m1 = m2;

  m1.publish();
  m2.publish();

  /******** MessageWrapper different type **********/

  MessageWrapper<test_t> m3;
  m3().msg = "m3";
  m3.publish();


  /******** ReactiveBuffer *********/

  std::cout << "*****" << std::endl;

  ReactiveBuffer monitoring("monitoring");
  ReactiveBuffer speaking("speaking");

  monitoring.setPriority(9);
  speaking.setPriority(6);

  m1 = "m1";
  monitoring.setData(&m1);
  speaking.setData(&m2);

  if(speaking > monitoring)
    speaking()->publish();
  else
    monitoring()->publish();

  monitoring.setData(&m3);

  if(speaking > monitoring)
    speaking()->publish();
  else
    monitoring()->publish();

  /********** ReactiveBufferStorage ********/

  std::cout << "*****" << std::endl;

  ReactiveBufferStorage buffers({"monitoring", "speaking"});

  buffers.setPriority("monitoring", 9);
  buffers.setPriority("speaking", 6);

  buffers["monitoring"]->setData(&m1);
  buffers["speaking"]->setData(&m2);

  buffers.getMorePriorityData()->publish();

  m1.setPriority(50);
  m2.setPriority(255);

  buffers.getMorePriorityData()->publish();

  m2.setPriority(0);
  buffers["monitoring"]->setData(&m3);

  buffers.getMorePriorityData()->publish();

  return 0;
}
