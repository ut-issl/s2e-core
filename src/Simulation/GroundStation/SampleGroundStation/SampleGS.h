#pragma once

#include <Component/CommGS/ANT.h>
#include <Component/CommGS/GScalculator.h>
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>

#include "../GroundStation.h"

class SampleGSComponents;

class SampleGS : public GroundStation {
 public:
  SampleGS(SimulationConfig* config, int gs_id);
  ~SampleGS();

  // 初期化
  virtual void Initialize(SimulationConfig* config);
  // ログ保存機能
  virtual void LogSetup(Logger& logger);
  // 状態量の更新
  virtual void Update(const Dynamics& dynamics, const GlobalEnvironment& global_env, const ANT& sc_ant, const SampleGS& samplegs,
                      const double& current_jd);

 private:
  SampleGSComponents* components_;
};
