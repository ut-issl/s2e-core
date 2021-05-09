#pragma once
#include <vector>
#include "../../Component/Abstract/ITickable.h"
#include "SimTime.h"

// ITickableを実装したクラスにクロック（Tick）を与えるクラス
// メインループなどで周期的にTickToComponentsを呼び出すという想定
class ClockGenerator
{
public:
  ~ClockGenerator();

  // クロックの周期（ミリ秒）
  const int IntervalMillisecond = 1;

  void RegisterComponent(ITickable* tickable);
  void RemoveComponent(ITickable* tickable);
  void TickToComponents();
  void UpdateComponents(const SimTime* sim_time);

  inline void ClearTimerCount(void) { timer_count_ = 0;}

private:
  std::vector<ITickable*> components_;
  unsigned long long timer_count_; 
};

