#pragma once

#include "../Spacecraft.h"
#include "SampleComponents.h"

class SampleComponents;

class SampleSat : public Spacecraft
{
public:
  SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  ~SampleSat();

  // 初期化
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  // ログ保存機能
  virtual void LogSetup(Logger& logger);
  // 状態量の更新
  virtual void Update(const SimTime* sim_time);

  //ダイナミクスへの力・トルク出力
  void GenerateTorque_b();
  void GenerateForce_b();

private:
  SampleComponents* components_;
};
