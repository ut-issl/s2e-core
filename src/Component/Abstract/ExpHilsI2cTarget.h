#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cTargetCommunicationBase.h"

// Example of the I2C Target side communication.
// This helps I2C communication for HILS testing.
// Supposed to be used in connection with I2C-USB Target converter: MFT200XD
// Data Sheet: https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT200XD.pdf
// telemetry size = 5 bytes(ASCII)
// Telemetry changes; ABCDE, BCDEF, ..., VWXYZ, ABCDE, ...

class ExpHilsI2cTarget : public ComponentBase, public ObcI2cTargetCommunicationBase
{
public:
  ExpHilsI2cTarget(
    const int prescaler,
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
  unsigned char tlm_counter_ = 0;
  const unsigned int kStoredFrameSize = 3;
  const unsigned char kNumAlphabet = 26;
};
