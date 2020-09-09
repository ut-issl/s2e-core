#include "ClockGenerator.h"

std::vector<ITickable*> ClockGenerator::components_;
unsigned long long ClockGenerator::timer_count_ = 0;

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
  for (auto itr = components_.begin(); itr != components_.end(); ++itr)
  {
    (*itr)->Tick(timer_count_);
  }
  timer_count_++;
}
