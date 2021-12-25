#include "ComponentBase.h"

#include "../../Dynamics/ClockGenerator.h"
#include "../../Interface/SpacecraftInOut/PowerDriver.h"

ComponentBase::ComponentBase() : ComponentBase(1)
{
}

ComponentBase::ComponentBase(int prescaler)
{
  ClockGenerator::RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;

  // デフォルトはON。
  // PowerPortと接続すると、まず電圧が初期化され、OFFになる。その後はPCUから制御されるのみ。
  // PowerPortのことを知らなければ、ONのまま使える。
  isOn_ = true;
}

// コピーされたらそのインスタンスもClockGeneratorに登録しないとならない
// 実用上、コピー元かコピー先のどちらか（大体コピー元？）はすぐに破棄されるはずだが
ComponentBase::ComponentBase(const ComponentBase & obj)
{
  prescaler_ = obj.prescaler_;
  isOn_ = obj.isOn_;
  ClockGenerator::RegisterComponent(this);
  PowerDriver::ReplaceComponent((ComponentBase*)&obj, this);
}

// 破棄されたらClockGeneratorから削除しないとならない
ComponentBase::~ComponentBase()
{
  ClockGenerator::RemoveComponent(this);
  PowerDriver::RemoveComponent(this);
}

void ComponentBase::SetPowerState(int port_id, double voltage)
{
  // 仮の実装
  // ポートIDによらず電圧が0以上ならこの機器をONにする、0ならオフにする
  isOn_ = voltage > 0;
}

double ComponentBase::GetCurrent(int port_id) const
{
  // 仮の実装
  // 消費電流は常に0 A
  return 0;
}

// 時を刻む
void ComponentBase::Tick(int count)
{
  if (!isOn_) return;
  if (count % prescaler_ > 0) return;
  MainRoutine(count);
}
