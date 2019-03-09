#ifndef ${project_name}_ARTIFICIALLIFE_H
#define ${project_name}_ARTIFICIALLIFE_H

#include "artificial_life/ArtificialLife.h"

class ${project_name}ArtificialLife : public ArtificialLife
{
public:
  ${project_name}ArtificialLife(std::shared_ptr<ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();
};

#endif // ${project_name}_ARTIFICIALLIFE_H
