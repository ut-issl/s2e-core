#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cTargetCommunicationBase.h"

// emulated components for training
// # Specification for ExpHilsI2cTarget
// * Checking I2C communication for HILS exam
//   Supposed to be used in connection with I2C-USB Target converter
//   telemetry size = 5 bytes(ASCII)
//   Telemetry changes; ABCDE, BCDEF, ..., VWXYZ, ABCDE, ...

class ExpHilsI2cTarget : public ComponentBase, public ObcI2cTargetCommunicationBase
{
public:
  ExpHilsI2cTarget(
    ClockGenerator* clock_gen,
    const int sils_port_id,
    unsigned char i2c_address,
    OBC* obc,
    const unsigned int hils_port_id,
    HilsPortManager* hils_port_manager
  );
  ~ExpHilsI2cTarget();
protected:
  void MainRoutine(int count);

private:
  unsigned char i2c_address_;
  unsigned int tlm_counter_ = 0;
  const unsigned int kStoredFrameSize = 3;
  const unsigned int kNumAlphabet = 26;
};
