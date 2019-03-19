#include "led_manager_test/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"

namespace led_manager_test {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100, buffer)
{
  on = false;
  cpt = 0;

  auto wrapped_OnOff_data = std::make_shared<resource_management::MessageWrapper<bool>>(on);

  wrapped_OnOff_data->setPriority(resource_management::useless);

  _buffer->setData(wrapped_OnOff_data);
}

void ArtificialLife::init()
{
  on = false;
  cpt = 0;

  auto wrapped_OnOff_data = std::make_shared<resource_management::MessageWrapper<bool>>(on);

  wrapped_OnOff_data->setPriority(resource_management::useless);

  _buffer->setData(wrapped_OnOff_data);
}

void ArtificialLife::inLoop()
{
  cpt++;
  if(cpt >= 100)
  {
    cpt = 0;
    on = !on;

    auto wrapped_OnOff_data = std::make_shared<resource_management::MessageWrapper<bool>>(on);
    wrapped_OnOff_data->setPriority(resource_management::useless);
    _buffer->setData(wrapped_OnOff_data);
  }
}

} // namespace led_manager_test
