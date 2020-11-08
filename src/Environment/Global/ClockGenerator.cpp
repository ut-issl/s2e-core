#include "ClockGenerator.h"

ClockGenerator::~ClockGenerator()
{
}

void ClockGenerator::RegisterComponent(ITickable* tickable)
{
  components_.push_back(tickable);
}

void ClockGenerator::RemoveComponent(ITickable* tickable)
{
  for (auto itr = components_.begin(); itr != components_.end();)
  {
    if (*itr == tickable)
    {
      components_.erase(itr++);
      break;
    }
    else
    {
      ++itr;
    }
  }
}

void ClockGenerator::TickToComponents()
{
  //Update for each component
  for (auto itr = components_.begin(); itr != components_.end(); ++itr) { (*itr)->Tick(timer_count_); }
  timer_count_++; //TODO: Consider if "timer_count" is necessary
}

void ClockGenerator::UpdateComponents(const SimTime* sim_time)
{
  if (sim_time->GetCompoUpdateFlag()) { TickToComponents(); }
}
