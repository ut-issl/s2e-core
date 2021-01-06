#pragma once
#include "OBC.h"

class OBC_C2A: public OBC
{
public:
  OBC_C2A(ClockGenerator* clock_gen);
  OBC_C2A(ClockGenerator* clock_gen, int timing_regulator);
  OBC_C2A(int prescaler, ClockGenerator* clock_gen, double current, int timing_regulator);
  ~OBC_C2A();

  // Communication port functions
  int ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size) override;
  int CloseComPort(int port_id) override;
  // OBC -> Components
  int SendFromObc (int port_id, unsigned char* buffer, int offset, int count) override;
  int ReceivedByCompo (int port_id, unsigned char* buffer, int offset, int count) override;
  // Components -> OBC
  int SendFromCompo(int port_id, unsigned char* buffer, int offset, int count) override;
  int ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count) override;
  // Static function for C2A
  static int SendFromObc_C2A(int port_id, unsigned char* buffer, int offset, int count);
  static int ReceivedByObc_C2A(int port_id, unsigned char* buffer, int offset, int count);
  
private:
  const int timing_regulator_;
  void MainRoutine(int count);
  void Initialize();
  // ports
  // Static function for C2A
  static std::map<int, SCIPort*> com_ports_c2a_;
};

// If the character encoding of C2A is UTF-8, these functions are not necessary, and users can directory use SendFromObc_C2A and ReceivedByObc_C2A
int OBC_C2A_SendFromObc(int port_id, unsigned char* buffer, int offset, int count);
int OBC_C2A_ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count);
