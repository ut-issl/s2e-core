/*
 * @file TMTCInterface.h
 * @brief Class for telemetry command communication with SILS GSTOS IF
 * @note TODO: Is this still needed? We can use normal serial communication port
 */

#pragma once
#include <msclr/gcroot.h>

#include "..\..\Interface\SpacecraftInOut\TMTCDriver.h"
#include "..\Abstract\ComponentBase.h"

class TMTCInterface : public ComponentBase {
 public:
  TMTCInterface(ClockGenerator* clock_gen, int port_id);
  ~TMTCInterface();

 protected:
  virtual void MainRoutine(int count);

 private:
  msclr::gcroot<TMTCDriver ^> tmtc_;
};
