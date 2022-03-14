#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcI2cControllerCommunicationBase.h"

// Example of the I2C Controller side communication.
// This helps I2C communication for HILS testing.
// Supposed to be used in connection with an I2C-USB Controller converter: SC18IM700
// Data Sheet: https://www.nxp.com/docs/en/data-sheet/SC18IM700.pdf
// telemetry size = 5 bytes(ASCII)

class ExpHilsI2cController : public ComponentBase, public ObcI2cControllerCommunicationBase
{
public:
  ExpHilsI2cController(
    const int prescaler,
    ClockGenerator* clock_gen,
    const unsigned int hils_port_id,
    const unsigned int baud_rate,
    const unsigned int tx_buf_size,
    const unsigned int rx_buf_size,
    HilsPortManager* hils_port_manager
  );
  ~ExpHilsI2cController();
protected:
  void MainRoutine(int count);
private:
  void RequestTlm();
  void Receive();
  static const uint8_t kCmdHeader_ = 0x53; // S
  static const uint8_t kReadCmd_   = 0x45;
  static const uint8_t kWriteCmd_  = 0x44;
  static const uint8_t kCmdFooter_ = 0x50; // P
};
