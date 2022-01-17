#pragma once
#include "../Dynamics/Dynamics.h"
#include "../Environment/Local/LocalEnvironment.h"
#include "Disturbance.h"

// The Abstract class for disturbance calculation with local environment and
// spacecraft dynamics トルクと並進力を発生する（どちらか片方だけでも良い）
class SimpleDisturbance : public Disturbance, public ILoggable {
public:
  virtual ~SimpleDisturbance() {}

  virtual inline void UpdateIfEnabled(const LocalEnvironment &local_env,
                                      const Dynamics &dynamics) {
    if (IsCalcEnabled) {
      Update(local_env, dynamics);
    } else {
      force_b_ *= 0;
      torque_b_ *= 0;
    }
  }

  // 環境とダイナミクスに応じて外乱のトルクと並進力を更新する
  virtual void Update(const LocalEnvironment &local_env,
                      const Dynamics &dynamics) = 0;
};

// 全ての外乱がこのインターフェースで計算できれば良いのだが、そういうわけにもいかない
// 例えば宇宙機間の万有引力は、相手と自分両方の軌道情報が必要（あまり考慮することはないだろうが……）
// そういった外乱については、Disturbanceを継承して適切な実装をすることになる
