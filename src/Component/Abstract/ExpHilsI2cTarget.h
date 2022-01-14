#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cTargetCommunicationBase.h"

// emulated components for training
// # Specification for ExpHilsI2cTarget
// * Checking I2C communication for HILS exam

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
  char i2c_address_;
};
