/**
 * @file SampleGS.cpp
 * @brief An example of user defined ground station class
 */

#include "SampleGS.h"

#include "SampleGSComponents.h"

SampleGS::SampleGS(SimulationConfig* config, int gs_id) : GroundStation(config, gs_id) { Initialize(config); }

SampleGS::~SampleGS() { delete components_; }

void SampleGS::Initialize(SimulationConfig* config) { components_ = new SampleGSComponents(config); }

void SampleGS::LogSetup(Logger& logger) {
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const Spacecraft& spacecraft, const GlobalEnvironment& global_env, const Antenna& sc_ant, const SampleGS& samplegs) {
  GroundStation::Update(global_env.GetCelesInfo().GetEarthRotation(), spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, sc_ant, samplegs, *(components_->GetAntenna()));
  // TODO: When compo->ant_ is null, the calculation is undefined.
}
