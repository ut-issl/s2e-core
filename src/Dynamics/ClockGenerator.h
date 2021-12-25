#pragma once
#include <vector>
#include "../Component/Abstract/ITickable.h"

// ITickableを実装したクラスにクロック（Tick）を与えるクラス
// メインループなどで周期的にTickToComponentsを呼び出すという想定
class ClockGenerator
{
public:
  ~ClockGenerator();

  // クロックの周期（ミリ秒）
  static const int IntervalMillisecond = 1;

  static void RegisterComponent(ITickable* tickable);
  static void RemoveComponent(ITickable* tickable);
  static void TickToComponents();

private:
  static std::vector<ITickable*> components_;
  static unsigned long long timer_count_;
};

