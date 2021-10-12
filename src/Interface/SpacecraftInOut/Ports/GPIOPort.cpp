#include "GPIOPort.h"

GPIOPort::GPIOPort(int port_id, IGPIOCompo* compo) : kPortId(port_id)
{
  hl_state_ = GPIO_LOW;
  component_ = compo;
}

GPIOPort::~GPIOPort()
{
}

int GPIOPort::DigitalWrite(bool isHigh)
{
  if (hl_state_ != isHigh)
  {
    // HIGH/LOWに変化があったら、割り込み関数を呼ぶ
    if (component_ != nullptr)
    {
      component_->GPIOStateChanged(kPortId, isHigh);
    }
  }
  hl_state_ = isHigh;
  return 0;
}

bool GPIOPort::DigitalRead()
{
  return hl_state_;
}
