#include "SCIDriver.h"
#include "SpacecraftInterface.h"

//FIXME: Static usage of this class is not a good pattern, because ports_ are
//created as pointers and must be freed at some point to avoid memory leaks.
std::map<int, SCIPort*> SCIDriver::ports_;

// Execute when the component is initialized on the simulated component
int SCIDriver::ConnectPort(int port_id, int tx_buf_size, int rx_buf_size)
{
  if (ports_[port_id] != nullptr)
  {
    // Port already used
    return -1;
  }
  ports_[port_id] = new SCIPort(tx_buf_size, rx_buf_size);
  return 0;
}

// Close port and free resources
int SCIDriver::ClosePort(int port_id)
{
  // Port not used
  if (ports_[port_id] == nullptr)
    return -1;

  SCIPort *port = ports_.at(port_id);
  delete port;
  ports_.erase(port_id);
}

int SCIDriver::SendToSim(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, count);
}

int SCIDriver::SendToSC(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteRx(buffer, offset, count);
}

int SCIDriver::ReceiveFromSim(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = ports_[port_id];
  if (port == nullptr) return -1;
  return port->ReadRx(buffer, offset, count);
}

int SCIDriver::ReceiveFromSC(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = ports_[port_id];
  if (port == nullptr) return -1;
  return port->ReadTx(buffer, offset, count);
}