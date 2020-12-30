#include "OBC.h"

OBC::OBC(ClockGenerator* clock_gen) : ComponentBase(1,clock_gen)
{
  Initialize();
}

OBC::OBC(int prescaler, ClockGenerator* clock_gen, double current)
: ComponentBase(prescaler,clock_gen), current_(current)
{
  Initialize();
}


OBC::~OBC()
{
}

void OBC::Initialize()
{
}

double OBC::GetCurrent(int port_id) const
{
  if (isOn_)
  {
    return current_;
  }
  return 0.0;
}

void OBC::MainRoutine(int count)
{
}

int OBC::ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size)
{
  if (com_ports_[port_id] != nullptr)
  {
    // Port already used
    return -1;
  }
  com_ports_[port_id] = new SCIPort(tx_buf_size, rx_buf_size);
  return 0;
}

// Close port and free resources
int OBC::CloseComPort(int port_id)
{
  // Port not used
  if (com_ports_[port_id] == nullptr)
    return -1;

  SCIPort *port = com_ports_.at(port_id);
  delete port;
  com_ports_.erase(port_id);
}

int OBC::SendFromObc(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, count);
}

int OBC::ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->ReadTx(buffer, offset, count);
}

int OBC::SendFromCompo(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteRx(buffer, offset, count);
}

int OBC::ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->ReadRx(buffer, offset, count);
}
