#pragma once
using namespace System;
using namespace System::ServiceModel;

[ServiceContract] interface class ITCTMChannel {
  [OperationContract] cli::array<Byte> ^ Cmd_to_SILS();

  [OperationContract] void Tlm_to_GSTOS(cli::array<Byte> ^ tlm_buf);
};
