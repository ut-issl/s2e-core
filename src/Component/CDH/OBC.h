#pragma once
#include "../Abstract/ComponentBase.h"
#include "../../Interface/SpacecraftInOut/Ports/SCIPort.h"
#include <map>

class OBC: public ComponentBase
{
public:
  OBC(ClockGenerator* clock_gen);
  OBC(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port);
  virtual ~OBC();

  // Communication port functions
  virtual int ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size);
  virtual int CloseComPort(int port_id);
  // OBC -> Components
  virtual int SendFromObc(int port_id, unsigned char* buffer, int offset, int count);
  virtual int ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int count);
  // Components -> OBC
  virtual int SendFromCompo(int port_id, unsigned char* buffer, int offset, int count);
  virtual int ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count);
  
protected:
  // function
  virtual void Initialize();
  virtual void MainRoutine(int count);
private:
  // ports
  std::map<int, SCIPort*> com_ports_;
};
