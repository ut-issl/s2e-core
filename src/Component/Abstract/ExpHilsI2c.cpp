#include "ExpHilsI2c.h"

ExpHilsI2c::ExpHilsI2c(
  ClockGenerator* clock_gen,
  const int sils_port_id,
  unsigned char i2c_address,
  OBC* obc,
  const unsigned int hils_port_id,
  HilsPortManager* hils_port_manager,
  const int mode_id)
  : ComponentBase(300, clock_gen),
  ObcI2cTargetCommunicationBase(sils_port_id, i2c_address, obc, hils_port_id, hils_port_manager), mode_id_(mode_id),
  i2c_address_(i2c_address)
{
}

ExpHilsI2c::~ExpHilsI2c()
{
}

void ExpHilsI2c::MainRoutine(int count)
{
  const int kWriteCmdSize = 7;
  const int kReadCmdSize = 4;
  const int kCmdSize = 3;
  const int kTlmSize = 3;

  if (mode_id_ == 0) // controller
  {
    const unsigned char read_tlm[kReadCmdSize] = { 'S', (unsigned char)(i2c_address_ + 1), (unsigned char)0x03, 'P'}; // read 3 bytes
    WriteRegister(0x00, read_tlm, kReadCmdSize);
    unsigned char rx_data[kTlmSize] = { 0 };
    ReadCommand(rx_data, kTlmSize);
    std::cout << "controller recieve: " << rx_data[0] << rx_data[1] << rx_data[2] << std::endl;
    const unsigned char write_tlm[kWriteCmdSize] = { 'S', (unsigned char)i2c_address_, (unsigned char)0x03, 'X', 'Y', 'Z', 'P'}; // XYZ
    WriteRegister(0x00, write_tlm, kWriteCmdSize);
  }
  else if (mode_id_ == 1) // target
  {
    unsigned char rx_data[kCmdSize] = { 0 };
    ReadCommand(rx_data, kCmdSize);
    std::cout << "target recieve: " << rx_data[0] << rx_data[1] << rx_data[2] << std::endl;
    const unsigned char tlm[kTlmSize] = { 'A', 'B', 'C'}; // ABC
    WriteRegister(0x00, tlm, kTlmSize);
  }
}
