#include "${project_name}/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"

namespace ${project_name} {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer)
{
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
!!for data_type in message_types
  // auto wrapped_{data_type[0]}_data = std::make_shared<resource_management::MessageWrapper<{data_type[2]}>>(data);
!!end
  //
  // 2 - Set the useless priority to your wrapped data:
!!for data_type in message_types
  // wrapped_{data_type[0]}_data->setPriority(resource_management::useless);
!!end
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
!!for data_type in message_types
  // _buffer->setData(wrapped_{data_type[0]}_data);
!!end
}

void ArtificialLife::init()
{
  // Put our own initialisation function here
  // It will be called each time a new artificial life cycle begins

  // Set an initial value in the artificial life buffer
  // if you do not do that, at each new cycle, the resource
  // will start with the previous artificial life value

  // Example:

  // 1 - Wrap your data with one of your types:
!!for data_type in message_types
  // auto wrapped_{data_type[0]}_data = std::make_shared<resource_management::MessageWrapper<{data_type[2]}>>(data);
!!end
  //
  // 2 - Set the useless priority to your wrapped data:
!!for data_type in message_types
  // wrapped_{data_type[0]}_data->setPriority(resource_management::useless);
!!end
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
!!for data_type in message_types
  // _buffer->setData(wrapped_{data_type[0]}_data);
!!end
}

void ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
!!for data_type in message_types
  // auto wrapped_{data_type[0]}_data = std::make_shared<resource_management::MessageWrapper<{data_type[2]}>>(data);
!!end
  //
  // 2 - Set the useless priority to your wrapped data:
!!for data_type in message_types
  // wrapped_{data_type[0]}_data->setPriority(resource_management::useless);
!!end
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
!!for data_type in message_types
  // _buffer->setData(wrapped_{data_type[0]}_data);
!!end
}

} // namespace ${project_name}
