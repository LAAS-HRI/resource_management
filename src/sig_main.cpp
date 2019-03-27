#include "resource_management/message_storage/MessageWrapper.h"
#include "resource_management/message_storage/ReactiveBuffer.h"
#include "resource_management/message_storage/ReactiveBufferStorage.h"

#include <iostream>
#include <thread>
#include <unistd.h>
#include <string>

struct test_t
{
  std::string msg;
};

void publishStrMsg(std::string msg, bool is_new)
{
  std::cout << "[StrMsg] " << msg << " " << is_new << std::endl;
}

void publishTestMsg(test_t msg, bool is_new)
{
  std::cout << "[TestMsg] "  << msg.msg << " " << is_new << std::endl;
}

int main()
{
  /******** MessageWrapper ******/
  resource_management::MessageWrapper<std::string>::registerPublishFunction(&publishStrMsg);
  resource_management::MessageWrapper<test_t>::registerPublishFunction(&publishTestMsg);

  resource_management::MessageWrapper<std::string> m1("m1");
  resource_management::MessageWrapper<std::string> m2;
  m2 = "m2";

  m1.publish();
  m2.publish();

  m1 = m2;

  m1.publish();
  m2.publish();

  /******** MessageWrapper different type **********/

  resource_management::MessageWrapper<test_t> m3;
  m3().msg = "m3";
  m3.publish();


  /******** ReactiveBuffer *********/

  std::cout << "*****" << std::endl;

  resource_management::ReactiveBuffer monitoring("monitoring");
  resource_management::ReactiveBuffer speaking("speaking");

  monitoring.setPriority(resource_management::prioritize);
  speaking.setPriority(resource_management::normal);

  m1 = "m1";
  monitoring.setData(std::make_shared<resource_management::MessageWrapper<std::string>>(m1));
  speaking.setData(std::make_shared<resource_management::MessageWrapper<std::string>>(m2));

  if(speaking.getPriority() > monitoring.getPriority())
    speaking()->publish();
  else
    monitoring()->publish();

  monitoring.setData(std::make_shared<resource_management::MessageWrapper<test_t>>(m3));

  if(speaking.getPriority() > monitoring.getPriority())
    speaking()->publish();
  else
    monitoring()->publish();

  /********** ReactiveBufferStorage ********/

  std::cout << "*****" << std::endl;

  resource_management::ReactiveBufferStorage buffers({"monitoring", "speaking"});

  buffers.setPriority("monitoring", resource_management::prioritize);
  buffers.setPriority("speaking", resource_management::normal);

  m1.setPriority(resource_management::helpful);
  m2.setPriority(resource_management::helpful);

  buffers["monitoring"]->setData(std::make_shared<resource_management::MessageWrapper<std::string>>(m1));
  buffers["speaking"]->setData(std::make_shared<resource_management::MessageWrapper<std::string>>(m2));

  buffers.getMorePriorityData()->publish();

  m2.setPriority(resource_management::vital);

  buffers.getMorePriorityData()->publish();

  m2.setPriority(resource_management::avoid);
  m3.setPriority(resource_management::helpful);
  buffers["monitoring"]->setData(std::make_shared<resource_management::MessageWrapper<test_t>>(m3));

  buffers.getMorePriorityData()->publish();

  return 0;
}
