#pragma once

#include "../Spacecraft.h"
#include "SampleComponents.h"

class SampleComponents;

class SampleSat : public Spacecraft
{
public:
  SampleSat(SimulationConfig config);
  ~SampleSat();

  // 初期化
  virtual void Initialize();
  // ログ保存機能
  virtual void LogSetup(Logger& logger);
  // 状態量の更新
  virtual void Update();

  //ダイナミクスへの力・トルク出力
  void GenerateTorque_b();
  void GenerateForce_b();

private:
  SampleComponents* components_;
};
