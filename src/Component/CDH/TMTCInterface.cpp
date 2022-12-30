/*
 * @file TMTCInterface.cpp
 * @brief Class for telemetry command communication with SILS GSTOS IF
 * @note TODO: Is this still needed? We can use normal serial communication port
 */

#include "TMTCInterface.h"

TMTCInterface::TMTCInterface(ClockGenerator* clock_gen, int port_id) : ComponentBase(100, clock_gen) { tmtc_ = gcnew TMTCDriver(port_id); }

TMTCInterface::~TMTCInterface() {}

void TMTCInterface::MainRoutine(int count) {
  tmtc_->ReceiveCommand();
  tmtc_->SendTelemetryIfAny();
}
