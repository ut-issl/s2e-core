#include "TMTCInterface.h"

TMTCInterface::TMTCInterface(int port_id) : ComponentBase(100)
{
  tmtc_ = gcnew TMTCDriver(port_id);
}

TMTCInterface::~TMTCInterface()
{
}

void TMTCInterface::MainRoutine(int count)
{
  tmtc_->ReceiveCommand();
  tmtc_->SendTelemetryIfAny();
}
