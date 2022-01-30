#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cControllerCommunicationBase.h"

// emulated components for training
// # Specification for ExpHilsI2cController
// * Checking I2C communication for HILS exam

class ExpHilsI2cController : public ComponentBase, public ObcI2cControllerCommunicationBase
{
public:
  ExpHilsI2cController(
    ClockGenerator* clock_gen,
    const unsigned int hils_port_id,
    HilsPortManager* hils_port_manager
  );
  ~ExpHilsI2cController();
protected:
  void MainRoutine(int count);
private:
  static const uint8_t kHeader_ = 0x53; // S
  static const uint8_t kRead_   = 0x45;
  static const uint8_t kWrite_  = 0x44;
  static const uint8_t kFooter_ = 0x50; // P
  int counter_ = 0;
};
