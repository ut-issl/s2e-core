#include "SampleGS.h"
#include "SampleGSComponents.h"

SampleGS::SampleGS(const SimulationConfig* config, int gs_id)
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
  components_ = new SampleGSComponents(config_);
}

void SampleGS::LogSetup(Logger& logger)
{
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const Dynamics& dynamics, const ANT& sc_ant, const SampleGS& samplegs, const double& current_jd)
{
  GroundStation::Update(current_jd);
  components_->gscalculator_->Update(dynamics, sc_ant, samplegs, *(components_->ant_));  // compo->ant_がnullの場合未定義動作になる気がするので対処が必要？
}
