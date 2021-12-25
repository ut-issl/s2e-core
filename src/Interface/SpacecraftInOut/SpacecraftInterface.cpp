#include "SpacecraftInterface.h"


#ifdef __cplusplus
#include "SCIDriver.h"
#include "GPIODriver.h"

int SendToSimWrapper(int port_id, unsigned char* buffer, int offset, int count)
{
  return SCIDriver::SendToSim(port_id, buffer, offset, count);
}
int ReceiveFromSimWrapper(int port_id, unsigned char* buffer, int offset, int count)
{
  return SCIDriver::ReceiveFromSim(port_id, buffer, offset, count);
}
int SendToScWrapper(int port_id, unsigned char* buffer, int offset, int count)
{
  return SCIDriver::SendToSC(port_id, buffer, offset, count);
}
int ReceiveFromScWrapper(int port_id, unsigned char* buffer, int offset, int count)
{
  return SCIDriver::ReceiveFromSC(port_id, buffer, offset, count);
}

int DigtalReadWrapper(int port_id)
{
  return GPIODriver::DigitalRead(port_id);
}

int DigtalWriteWrapper(int port_id, bool isHigh)
{
  return GPIODriver::DigitalWrite(port_id, isHigh);
}

#endif

int send_to_sim(int port_id, unsigned char* buffer, int offset, int count)
{
  return SendToSimWrapper(port_id, buffer, offset, count);
}

int receive_from_sim(int port_id, unsigned char* buffer, int offset, int count)
{
  return ReceiveFromSimWrapper(port_id, buffer, offset, count);
}

int send_to_sc(int port_id, unsigned char* buffer, int offset, int count)
{
  return SendToScWrapper(port_id, buffer, offset, count);
}

int receive_from_sc(int port_id, unsigned char* buffer, int offset, int count)
{
  return ReceiveFromScWrapper(port_id, buffer, offset, count);
}

int digital_read(int port_id)
{
  return DigtalReadWrapper(port_id);
}

int digital_write(int port_id, bool isHigh)
{
  return DigtalWriteWrapper(port_id, isHigh);
}
