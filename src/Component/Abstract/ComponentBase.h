#pragma once
#include "ITickable.h"
#include "../../Environment/Global/ClockGenerator.h"
#include "../../Interface/SpacecraftInOut/Ports/PowerPort.h"

// 電源ON/OFFと時間の概念のみを持った、コンポーネントの基底クラス
class ComponentBase : public ITickable
{
public:
  ComponentBase(int prescaler, ClockGenerator* clock_gen);
  ComponentBase(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port);
  ComponentBase(const ComponentBase& obj);
  virtual ~ComponentBase();

  // クロック入力を受けるメソッド
  // 外部から周期的に呼び出すという前提
  virtual void Tick(int count);

protected:
  // クロックの分周率
  // クロックは他所で指定された周波数で入力される
  // コンポの動作周波数がそれよりも低い場合は、分周するとクロック周波数を下げることができる
  // TODO:基本の周期は確定させたほうが良いだろう
  int prescaler_;

  // クロックを分周した周期で呼び出されるメソッド
  // 周期的な処理をここに書く（定期テレメ送信・コマンド受信など）
  virtual void MainRoutine(int time_count) = 0;

  ClockGenerator* clock_gen_;
  PowerPort* power_port_;
};

