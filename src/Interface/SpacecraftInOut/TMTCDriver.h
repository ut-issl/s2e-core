/**
 * @file TMTCDriver.h
 * @brief Driver of TMTC
 */

#pragma once
//#include "../../../stdafx.h"
#include "SCIDriver.h"
#include "Utils/ITCTMChannel.h"

using namespace System;
using namespace System::ServiceModel;
using namespace System::ServiceModel::Channels;

/**
 * @class TMTCDriver
 * @brief Driver of TMTC
 * @details Execute the process communication with SILS_GSTOS_IF to send telemetry and receive command
 * @details Managed class for IPC. You can use as a native class by using gcroot.
 */
ref class TMTCDriver {
 public:
  TMTCDriver(int port_id);
  void ReceiveCommand();
  void SendTelemetryIfAny();

 private:
  ITCTMChannel ^ tctm_if_;
  const unsigned char kPortId = 7;

  void Initialize();
};
