#pragma once

#include <map>
#include "Ports/GPIOPort.h"
#include "../../Component/Abstract/IGPIOCompo.h"

class GPIODriver
{
public:
  // 指定したポートIDでGPIOポートを開く
  // 立ち上がり/立ち下がり時に割り込みが必要なら（模擬コンポ側のみ）、
  // 模擬コンポ側でIGPIOCompoを実装して、模擬コンポのインスタンスへのポインタを引数に渡す
  // ポートが既に使われていた場合は、-1が返る
  static int ConnectPort(int port_id, IGPIOCompo* compo = nullptr);

  // port_idのGPIOポートにHIGH/LOWを書き込む
  // isHighが真ならHIGH、偽ならLOW
  static int DigitalWrite(int port_id, bool isHigh);

  // port_idのGPIOポートから、HIGH/LOWを読み込む
  // 返り値が真ならHIGH、偽ならLOW
  static bool DigitalRead(int port_id);

private:
  static std::map<int, GPIOPort*> ports_;
};

