#ifndef MESSAGEABSTRACTION_H
#define MESSAGEABSTRACTION_H

class MessageAbstraction
{
public:
  virtual ~MessageAbstraction() {}

  virtual void publish() = 0;
};

#endif // MESSAGEABSTRACTION_H
