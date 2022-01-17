#include "ComponentBase.h"

ComponentBase::ComponentBase(int prescaler, ClockGenerator *clock_gen,
                             int fast_prescaler)
    : clock_gen_(clock_gen) {
  power_port_ = new PowerPort();
  clock_gen_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

ComponentBase::ComponentBase(int prescaler, ClockGenerator *clock_gen,
                             PowerPort *power_port, int fast_prescaler)
    : clock_gen_(clock_gen), power_port_(power_port) {
  clock_gen_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

// コピーされたらそのインスタンスもClockGeneratorに登録しないとならない
// 実用上、コピー元かコピー先のどちらか（大体コピー元？）はすぐに破棄されるはずだが
ComponentBase::ComponentBase(const ComponentBase &obj) {
  prescaler_ = obj.prescaler_;
  fast_prescaler_ = obj.fast_prescaler_;
  needs_fast_update_ = obj.needs_fast_update_;
  clock_gen_ = obj.clock_gen_;
  clock_gen_->RegisterComponent(this);
  power_port_ = obj.power_port_;
}

// 破棄されたらClockGeneratorから削除しないとならない
ComponentBase::~ComponentBase() { clock_gen_->RemoveComponent(this); }

// 時を刻む
void ComponentBase::Tick(int count) {
  if (!power_port_->GetIsOn())
    return;
  if (count % prescaler_ > 0)
    return;
  MainRoutine(count);
}

void ComponentBase::FastTick(int count) {
  if (!power_port_->GetIsOn())
    return;
  if (count % fast_prescaler_ > 0)
    return;
  FastUpdate();
}
