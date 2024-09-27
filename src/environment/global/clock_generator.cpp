/**
 * @file clock_generator.cpp
 * @brief Class to generate clock for classes which have ITickable
 */

#include "clock_generator.hpp"

namespace s2e::environment {

ClockGenerator::~ClockGenerator() {}

void ClockGenerator::RegisterComponent(ITickable* tickable) { components_.push_back(tickable); }

void ClockGenerator::RemoveComponent(ITickable* tickable) {
  for (auto itr = components_.begin(); itr != components_.end();) {
    if (*itr == tickable) {
      components_.erase(itr++);
      break;
    } else {
      ++itr;
    }
  }
}

void ClockGenerator::TickToComponents() {
  // Update for each component
  for (auto itr = components_.begin(); itr != components_.end(); ++itr) {
    // Run MainRoutine
    (*itr)->Tick(timer_count_);
    // Run FastUpdate (Processes that are executed more frequently than MainRoutine)
    if ((*itr)->GetNeedsFastUpdate()) {
      (*itr)->FastTick(timer_count_);
    }
  }
  timer_count_++;  // TODO: Consider if "timer_count" is necessary
}

void ClockGenerator::UpdateComponents(const SimulationTime* simulation_time) {
  if (simulation_time->GetCompoUpdateFlag()) {
    TickToComponents();
  }
}

} // namespace s2e::environment
