/**
 * @file component.cpp
 * @brief Base class for component emulation. All components have to inherit this.
 */

#include "component.hpp"

ComponentBase::ComponentBase(const unsigned int prescaler, ClockGenerator* clock_generator, const unsigned int fast_prescaler)
    : clock_generator_(clock_generator) {
  power_port_ = new PowerPort();
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

ComponentBase::ComponentBase(const unsigned int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const unsigned int fast_prescaler)
    : clock_generator_(clock_generator), power_port_(power_port) {
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

ComponentBase::ComponentBase(const ComponentBase& object) {
  prescaler_ = object.prescaler_;
  fast_prescaler_ = object.fast_prescaler_;
  needs_fast_update_ = object.needs_fast_update_;
  clock_generator_ = object.clock_generator_;
  clock_generator_->RegisterComponent(this);
  power_port_ = object.power_port_;
}

ComponentBase::~ComponentBase() { clock_generator_->RemoveComponent(this); }

void ComponentBase::Tick(const unsigned int count) {
  if (count % prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    MainRoutine(count);
  } else {
    PowerOffRoutine();
  }
}

void ComponentBase::FastTick(const unsigned int fast_count) {
  if (fast_count % fast_prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    FastUpdate();
  } else {
    PowerOffRoutine();
  }
}
