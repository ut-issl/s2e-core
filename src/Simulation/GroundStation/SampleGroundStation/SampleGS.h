#pragma once

#include "../GroundStation.h"
#include "../../Spacecraft/SampleSpacecraft/SampleSat.h"

class SampleGSComponents;

class SampleGS : public GroundStation
{
public:
  SampleGS(SimulationConfig config, int gs_id);
  ~SampleGS();

  // 初期化
  virtual void Initialize();
  // ログ保存機能
  virtual void LogSetup(Logger& logger);
  // 状態量の更新
  virtual void Update(const SampleSat& samplesat, const ANT& sc_ant, const SampleGS& samplegs);

private:
  SampleGSComponents* components_;
};
