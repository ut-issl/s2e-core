#pragma once
#include "../../../Component/Abstract/IGPIOCompo.h"

#define GPIO_HIGH true
#define GPIO_LOW false

class GPIOPort {
public:
  GPIOPort(int port_id_, IGPIOCompo *compo = nullptr);
  ~GPIOPort();
  int DigitalWrite(bool isHigh);
  bool DigitalRead();

private:
  const int kPortId;
  IGPIOCompo *component_;
  bool hl_state_;
};
