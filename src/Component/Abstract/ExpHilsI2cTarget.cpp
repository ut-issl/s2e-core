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
}

ExpHilsI2cTarget::~ExpHilsI2cTarget()
{
}

void ExpHilsI2cTarget::MainRoutine(int count)
{
  // update telemetry data
  const unsigned int kTlmSize = 5;
  unsigned char tlm[kTlmSize] = { 0 };
  for (int i = 0; i < kTlmSize; i++)
  {
    tlm[i] = (char)('A' + tlm_counter_ + i);
  }
  WriteRegister(0, tlm, kTlmSize);
  tlm_counter_++;
  if (tlm_counter_ > kNumAlphabet - kTlmSize)
  {
    tlm_counter_ = 0; // ABCDE, ..., VWXYZ, ABCDE, ...
  }

  ReceiveCommand();
  int add_frame_counter = kStoredFrameSize - GetStoredFrameCounter();
  if (add_frame_counter <= 0) return;

  // store telemetry in converter up to kStoredFrameSize
  for (int i = 0; i < add_frame_counter; i++)
  {
    SendTelemetry(kTlmSize);
    std::cout << "Target sends to converter: " << tlm[0] << tlm[1] << tlm[2] << tlm[3] << tlm[4] << std::endl;
  }

  return;
}
