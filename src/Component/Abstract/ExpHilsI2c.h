#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cCommunicationBase.h"

// emulated components for training
// # Specification for ExpHilsI2c
// * Checking I2C communication for HILS exam
//   compo_id = 0 : the master compo which sends out "XYZ"
//   compo_id = 1 : the slave compo which returns "ABC"

class ExpHilsI2c : public ComponentBase, public ObcI2cCommunicationBase
{
public:
  ExpHilsI2c(
    ClockGenerator* clock_gen,
    const int sils_port_id,
    unsigned char i2c_address,
    OBC* obc,
    const unsigned int hils_port_id,
    HilsPortManager* hils_port_manager,
    const int mode_id
  );
  ~ExpHilsI2c();
protected:
  void MainRoutine(int count);

private:
  const int mode_id_;
  char i2c_address_;
};
