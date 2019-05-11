#ifndef ${project_name}_ARTIFICIALLIFE_H
#define ${project_name}_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"

namespace ${project_name} {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();
};

} // namespace ${project_name}

#endif // ${project_name}_ARTIFICIALLIFE_H
