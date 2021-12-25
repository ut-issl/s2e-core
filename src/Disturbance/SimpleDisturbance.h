#pragma once
#include "../Environment/Environment.h"
#include "Disturbance.h"
#include "../Simulation/Spacecraft/Spacecraft.h"

// 環境とダイナミクスのみから計算できる単純な外乱の抽象クラス
// トルクと並進力を発生する（どちらか片方だけでも良い）
class SimpleDisturbance : public Disturbance, public ILoggable
{
public:
  virtual ~SimpleDisturbance() { }

  virtual inline void UpdateIfEnabled(Envir& env, const Spacecraft& spacecraft)
  {
    if (IsCalcEnabled) { Update(env, spacecraft); }
    else { force_b_ *= 0; torque_b_ *= 0; }
  }

  // 環境とダイナミクスに応じて外乱のトルクと並進力を更新する
  virtual void Update(Envir& env, const Spacecraft& spacecraft) = 0;
};

// 全ての外乱がこのインターフェースで計算できれば良いのだが、そういうわけにもいかない
// 例えば宇宙機間の万有引力は、相手と自分両方の軌道情報が必要（あまり考慮することはないだろうが……）
// そういった外乱については、Disturbanceを継承して適切な実装をすることになる
