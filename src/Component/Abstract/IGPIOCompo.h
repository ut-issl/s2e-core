#pragma once
class IGPIOCompo {
public:
  virtual ~IGPIOCompo(){};
  // GPIOのHIGH/LOWが変化したときに呼び出される
  // GPIOはしばしば割り込み的に使われるので、用意した
  // ポートにより振る舞いを変更する必要がある可能性があるので、引数にport_idを含める
  virtual void GPIOStateChanged(int port_id, bool isPosedge) = 0;
};
