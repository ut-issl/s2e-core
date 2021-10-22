#pragma once
#include <vector>
#include "ComponentBase.h"
#include "ObcCommunicationBase.h"

// mock components for training
// # Specification for ExpHils
// * Checking UART communication for HILS exam
//   compo_id = 0 : the sender compo which sends out a new massage
//   compo_id = 1 : the responder compo which returns the message as received
// * message size is 4 bytes
//   the first 3 bytes : ASCII(ABC, BCD, CDE,...)
//   the last byte : \0

class ExpHils : public ComponentBase, public ObcCommunicationBase
{
public:
  ExpHils(
    ClockGenerator* clock_gen,
    const int sils_port_id,
    OBC* obc,
    const unsigned int hils_port_id,
    const unsigned int baud_rate,
    HilsPortManager* hils_port_manager,
    const int mode_id
  );
  ~ExpHils();
protected:
  void MainRoutine(int count);

private:
  const static int kMemorySize = 4;
  const int kNumAlphabet = 26;
  char memory_[kMemorySize] = { 0, 0, 0, '\0' }; // for the responder compo
  char tx_[kMemorySize] = { 0, 0, 0, '\0' };
  const int mode_id_;
  int counter_ = 0;

  // override ObcComunication
  int ParseCommand(const int cmd_size) override;
  int GenerateTelemetry() override;
};
