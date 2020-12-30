#pragma once
#include <vector>
#include "ComponentBase.h"
#include "IGPIOCompo.h"
#include "../CDH/OBC_C2A.h"

// 研修用の模擬コンポーネント
// # コンポ「EXP」の仕様
// * コマンドは5バイト:
//   * 最初の3バイト: “SET”
//   * 次の1バイト: セットするデータ ASCIIの0x21~0x7e
//   * 次の1バイト: “\n”
// * テレメは100バイト:
//   * 今までSETされたデータの蓄積を返す
//   * SETされたデータが100バイトを超えると、古い方から切り捨てていく
//   * 空のときは\0で埋められている
//   * 100バイト目は必ず ’\n’
// * テレメは定期的に自動で送出される
// * その他の仕様は都合良く仮定して良い
class EXP : public ComponentBase, public IGPIOCompo
{
public:
  EXP(ClockGenerator* clock_gen, int port_id, OBC* obc);
  ~EXP();
  int ReceiveCommand();
  int SendTelemetry();
protected:
  void MainRoutine(int count);
  void GPIOStateChanged(int port_id, bool isPosedge);
  double GetCurrent(int port_id) const;
private:
  OBC* obc_;
  const static int MAX_MEMORY_LEN = 100;
  std::vector<char> memory;
  char memoryc[100];
  int port_id_;
  unsigned char tx_buff[MAX_MEMORY_LEN];
  unsigned char rx_buff[MAX_MEMORY_LEN];

  int ParseCommand(unsigned char* cmd);
  int Initialize();
};