#ifndef led_manager_ARTIFICIALLIFE_H
#define led_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"

namespace led_manager_test {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();

  bool on;
  int cpt;
};

} // namespace led_manager_test

#endif // led_manager_ARTIFICIALLIFE_H
