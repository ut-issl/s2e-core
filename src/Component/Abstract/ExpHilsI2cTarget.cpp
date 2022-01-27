#include "ExpHilsI2cTarget.h"

ExpHilsI2cTarget::ExpHilsI2cTarget(
  ClockGenerator* clock_gen,
  const int sils_port_id,
  unsigned char i2c_address,
  OBC* obc,
  const unsigned int hils_port_id,
  HilsPortManager* hils_port_manager)
  : ComponentBase(1, clock_gen),
  ObcI2cTargetCommunicationBase(sils_port_id, i2c_address, obc, hils_port_id, hils_port_manager),
  i2c_address_(i2c_address)
{
  const unsigned int kRegSize = 5;
  unsigned char rx_data[kRegSize] = { 0 };
  for (int i = 0; i < kRegSize; i++)
  {
    rx_data[i] = 'A' + i;
  }
  WriteRegister(0, rx_data, kRegSize);
}

ExpHilsI2cTarget::~ExpHilsI2cTarget()
{
}

void ExpHilsI2cTarget::MainRoutine(int count)
{
  ReceiveCommand();
  int ret = CheckFlag();
  if (ret >= 0) // レジスタアドレスに応じてテレメ長を指定
  {
    SendTelemetry(5);
  }

  const unsigned int kRegSize = 5;
  unsigned char rx_data[kRegSize] = { 0 };
  ReadRegister(0, rx_data, kRegSize);
  std::cout << "register: " << rx_data[0] << rx_data[1] << rx_data[2] << rx_data[3] << rx_data[4] << "\n" << std::endl;
}
