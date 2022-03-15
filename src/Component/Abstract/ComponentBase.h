#pragma once
#include <Environment/Global/ClockGenerator.h>
#include <Interface/SpacecraftInOut/Ports/PowerPort.h>

#include "ITickable.h"

// 電源ON/OFFと時間の概念のみを持った、コンポーネントの基底クラス
class ComponentBase : public ITickable {
 public:
  ComponentBase(int prescaler, ClockGenerator* clock_gen,
                int fast_prescaler = 1);
  ComponentBase(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port,
                int fast_prescaler = 1);
  ComponentBase(const ComponentBase& obj);
  virtual ~ComponentBase();

  // クロック入力を受けるメソッド
  // 外部から周期的に呼び出すという前提
  virtual void Tick(int count);
  virtual void FastTick(int fast_count);

 protected:
  // クロックの分周率
  // クロックは他所で指定された周波数で入力される
  // コンポの動作周波数がそれよりも低い場合は、分周するとクロック周波数を下げることができる
  // TODO:基本の周期は確定させたほうが良いだろう
  int prescaler_;
  //  Frequency scale factor for fast update
  int fast_prescaler_ = 1;

  // クロックを分周した周期で呼び出されるメソッド
  // 周期的な処理をここに書く（定期テレメ送信・コマンド受信など）
  virtual void MainRoutine(int time_count) = 0;

  // Method used to calculate high-frequency disturbances(e.g. RW jitter)
  // Override only when high-frequency disturbances need to be calculated.
  virtual void FastUpdate(){};

  ClockGenerator* clock_gen_;
  PowerPort* power_port_;
};
