/**
 * @file sample_ground_station.cpp
 * @brief An example of user defined ground station class
 */

#include "sample_ground_station.hpp"

#include "SampleGSComponents.h"

SampleGS::SampleGS(SimulationConfig* config, int gs_id) : GroundStation(config, gs_id) { Initialize(config); }

SampleGS::~SampleGS() { delete components_; }

void SampleGS::Initialize(SimulationConfig* config) { components_ = new SampleGSComponents(config); }

void SampleGS::LogSetup(Logger& logger) {
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const CelestialRotation& celes_rotation, const SampleSat& spacecraft) {
  GroundStation::Update(celes_rotation, spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, spacecraft.GetInstalledComponents().GetAntenna(), *this, *(components_->GetAntenna()));
}
