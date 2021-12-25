#pragma once
#include <msclr/gcroot.h>
#include "..\Abstract\ComponentBase.h"
#include "..\..\Interface\SpacecraftInOut\TMTCDriver.h"

// SILS_GSTOS_IFからのテレコマ送受信を行うクラス
// 宇宙機に搭載されるコンポーネントではないが、まあ、便宜上そうした
class TMTCInterface : public ComponentBase
{
public:
  TMTCInterface(int port_id);
  ~TMTCInterface();
protected:
  virtual void MainRoutine(int count);
private:
  msclr::gcroot<TMTCDriver^> tmtc_;
};
