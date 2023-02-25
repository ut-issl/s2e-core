/**
 * @file component.cpp
 * @brief Base class for component emulation. All components have to inherit this.
 */

#include "component.hpp"

ComponentBase::ComponentBase(const int prescaler, ClockGenerator* clock_gen, const int fast_prescaler) : clock_generator_(clock_gen) {
  power_port_ = new PowerPort();
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

ComponentBase::ComponentBase(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const int fast_prescaler)
    : clock_generator_(clock_gen), power_port_(power_port) {
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

ComponentBase::ComponentBase(const ComponentBase& obj) {
  prescaler_ = obj.prescaler_;
  fast_prescaler_ = obj.fast_prescaler_;
  needs_fast_update_ = obj.needs_fast_update_;
  clock_generator_ = obj.clock_generator_;
  clock_generator_->RegisterComponent(this);
  power_port_ = obj.power_port_;
}

ComponentBase::~ComponentBase() { clock_generator_->RemoveComponent(this); }

void ComponentBase::Tick(const int count) {
  if (count % prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    MainRoutine(count);
  } else {
    PowerOffRoutine();
  }
}

void ComponentBase::FastTick(const int count) {
  if (count % fast_prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    FastUpdate();
  } else {
    PowerOffRoutine();
  }
}
