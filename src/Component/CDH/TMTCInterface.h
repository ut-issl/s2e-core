#pragma once
#include <msclr/gcroot.h>

#include "..\..\Interface\SpacecraftInOut\TMTCDriver.h"
#include "..\Abstract\ComponentBase.h"

// SILS_GSTOS_IFからのテレコマ送受信を行うクラス
// 宇宙機に搭載されるコンポーネントではないが、まあ、便宜上そうした
class TMTCInterface : public ComponentBase {
 public:
  TMTCInterface(ClockGenerator* clock_gen, int port_id);
  ~TMTCInterface();

 protected:
  virtual void MainRoutine(int count);

 private:
  msclr::gcroot<TMTCDriver ^> tmtc_;
};
