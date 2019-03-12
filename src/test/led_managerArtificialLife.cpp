#include "led_managerArtificialLife.h"
#include "message_storage/MessageWrapper.h"

#include "led_manager/Color.h"
#include "led_manager/OnOff.h"

led_managerArtificialLife::led_managerArtificialLife(std::shared_ptr<ReactiveBuffer> buffer) :
        ArtificialLife(100, buffer)
{
  on = false;
  cpt = 0;

  auto wrapped_OnOff_data = std::make_shared<MessageWrapper<bool>>(on);

  wrapped_OnOff_data->setPriority(useless);

  _buffer->setData(wrapped_OnOff_data);
}

void led_managerArtificialLife::init()
{
  on = false;
  cpt = 0;

  auto wrapped_OnOff_data = std::make_shared<MessageWrapper<bool>>(on);

  wrapped_OnOff_data->setPriority(useless);

  _buffer->setData(wrapped_OnOff_data);
}

void led_managerArtificialLife::inLoop()
{
  cpt++;
  if(cpt >= 100)
  {
    cpt = 0;
    on = !on;
  }

  led_manager::OnOff data;

  auto wrapped_OnOff_data = std::make_shared<MessageWrapper<bool>>(on);

  wrapped_OnOff_data->setPriority(useless);

  _buffer->setData(wrapped_OnOff_data);
}
