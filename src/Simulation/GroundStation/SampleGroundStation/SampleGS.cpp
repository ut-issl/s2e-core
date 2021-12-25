#include "SampleGS.h"
#include "SampleGSComponents.h"

SampleGS::SampleGS(SimulationConfig config, int gs_id)
  :GroundStation(config, gs_id)
{
  Initialize();
}

SampleGS::~SampleGS()
{
  delete components_;
}

void SampleGS::Initialize()
{
  components_ = new SampleGSComponents(&config_);
}

void SampleGS::LogSetup(Logger& logger)
{
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const SampleSat& samplesat, const ANT& sc_ant, const SampleGS& samplegs)
{
  GroundStation::Update();
  components_->gscalculator_->Update(samplesat, sc_ant, samplegs, *(components_->ant_));  // compo->ant_がnullの場合未定義動作になる気がするので対処が必要？
}
