/**
 * @file component.cpp
 * @brief Base class for component emulation. All components have to inherit this.
 */

#include "component.hpp"

Component::Component(const unsigned int prescaler, ClockGenerator* clock_generator, const unsigned int fast_prescaler)
    : clock_generator_(clock_generator) {
  power_port_ = new PowerPort();
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

Component::Component(const unsigned int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const unsigned int fast_prescaler)
    : clock_generator_(clock_generator), power_port_(power_port) {
  clock_generator_->RegisterComponent(this);
  prescaler_ = (prescaler > 0) ? prescaler : 1;
  fast_prescaler_ = (fast_prescaler > 0) ? fast_prescaler : 1;
}

Component::Component(const Component& object) {
  prescaler_ = object.prescaler_;
  fast_prescaler_ = object.fast_prescaler_;
  needs_fast_update_ = object.needs_fast_update_;
  clock_generator_ = object.clock_generator_;
  clock_generator_->RegisterComponent(this);
  power_port_ = object.power_port_;
}

Component::~Component() { clock_generator_->RemoveComponent(this); }

void Component::Tick(const unsigned int count) {
  if (count % prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    MainRoutine(count);
  } else {
    PowerOffRoutine();
  }
}

void Component::FastTick(const unsigned int fast_count) {
  if (fast_count % fast_prescaler_ > 0) return;
  if (power_port_->GetIsOn()) {
    FastUpdate();
  } else {
    PowerOffRoutine();
  }
}
