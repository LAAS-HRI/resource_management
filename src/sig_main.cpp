#include "message_storage/MessageWrapper.h"

#include <iostream>
#include <thread>
#include <unistd.h>
#include <string>

void publishStrMsg(std::string msg)
{
  std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
  MessageWrapper<std::string> m1("m1");
  MessageWrapper<std::string> m2;
  m2 = "m2";

  m1.registerPublishFunction(&publishStrMsg);
  m2.registerPublishFunction(&publishStrMsg);

  m1.publish();
  m2.publish();

  m1 = m2;

  m1.publish();
  m2.publish();

  return 0;
}
