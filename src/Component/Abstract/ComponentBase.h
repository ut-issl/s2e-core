#pragma once
#include "ITickable.h"
#include "../../Environment/Global/ClockGenerator.h"

// 電源ON/OFFと時間の概念のみを持った、コンポーネントの基底クラス
class ComponentBase : public ITickable
{
public:
  ComponentBase(int prescaler, ClockGenerator* clock_gen);
  ComponentBase(const ComponentBase& obj);
  virtual ~ComponentBase();

  // 各電源ポートの電圧変更を反映する
  virtual void SetPowerState(int port_id, double voltage);

  // port_idの電源ポートにおける消費電流[A]を返す
  // PCUのテレメを現実的なものにするため、子クラスでのoverrideを推奨する
  virtual double GetCurrent(int port_id) const;

  // クロック入力を受けるメソッド
  // 外部から周期的に呼び出すという前提
  virtual void Tick(int count);

private:
  // クロックの分周率
  // クロックは他所で指定された周波数で入力される
  // コンポの動作周波数がそれよりも低い場合は、分周するとクロック周波数を下げることができる
  // TODO:基本の周期は確定させたほうが良いだろう
  int prescaler_;

protected:
  // 電源のON/OFFを示す変数
  // 複数系統の電源が接続される機器に関しては、主電源のみに関するON/OFFとみなす
  // isOn_ == falseならクロック入力が無視されるようになる（時が止まる）
  bool isOn_;

  // クロックを分周した周期で呼び出されるメソッド
  // 周期的な処理をここに書く（定期テレメ送信・コマンド受信など）
  virtual void MainRoutine(int time_count) = 0;

  // 
  ClockGenerator* clock_gen_;
};

