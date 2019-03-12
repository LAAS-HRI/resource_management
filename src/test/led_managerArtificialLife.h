#ifndef led_manager_ARTIFICIALLIFE_H
#define led_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"

class led_managerArtificialLife : public ArtificialLife
{
public:
  led_managerArtificialLife(std::shared_ptr<ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();

  bool on;
  int cpt;
};

#endif // led_manager_ARTIFICIALLIFE_H
