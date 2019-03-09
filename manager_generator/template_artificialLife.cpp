#include "${project_name}ArtificialLife.h"

${project_name}ArtificialLife::${project_name}ArtificialLife(std::shared_ptr<ReactiveBuffer> buffer) :
        ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer)
{

}

void ${project_name}ArtificialLife::init()
{
  // Put our own initialisation function here
  // It will be called each time a new artificial life cycle begins
}

void ${project_name}ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
!!for data_type in messages_types_zip
  // auto wrapped_{data_type[0]}_data = std::make_shared<MessageWrapper<{data_type[2]}>>(data);
!!end
  //
  // 2 - Set the useless priority to your wrapped data:
!!for data_type in messages_types_zip
  // wrapped_{data_type[0]}_data->setPriority(useless);
!!end
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
!!for data_type in messages_types_zip
  // _buffer->setData(wrapped_{data_type[0]}_data);
!!end
}
