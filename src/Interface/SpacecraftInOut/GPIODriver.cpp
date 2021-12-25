#include "GPIODriver.h"
//#include "TMTCDriver.h"

std::map<int, GPIOPort*> GPIODriver::ports_;

int GPIODriver::ConnectPort(int port_id, IGPIOCompo * compo)
{
  if (ports_[port_id] != nullptr)
  {
    // 既に使用されているポートを指定された
    return -1;
  }
  ports_[port_id] = new GPIOPort(port_id, compo);
  return 0;
}

int GPIODriver::DigitalWrite(int port_id, bool isHigh)
{
  GPIOPort* port = ports_[port_id];
  if (port == nullptr) return -1;
  return port->DigitalWrite(isHigh);
}

bool GPIODriver::DigitalRead(int port_id)
{
  GPIOPort* port = ports_[port_id];
  if (port == nullptr) return false;  // to be revised
  return port->DigitalRead();
}
