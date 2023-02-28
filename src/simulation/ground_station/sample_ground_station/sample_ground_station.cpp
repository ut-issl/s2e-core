/**
 * @file sample_ground_station.cpp
 * @brief An example of user defined ground station class
 */

#include "sample_ground_station.hpp"

#include "sample_ground_station_components.hpp"

SampleGS::SampleGS(SimulationConfig* configuration, int gs_id) : GroundStation(configuration, gs_id) { Initialize(configuration); }

SampleGS::~SampleGS() { delete components_; }

void SampleGS::Initialize(SimulationConfig* configuration) { components_ = new SampleGSComponents(configuration); }

void SampleGS::LogSetup(Logger& logger) {
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const CelestialRotation& celes_rotation, const SampleSat& spacecraft) {
  GroundStation::Update(celes_rotation, spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, spacecraft.GetInstalledComponents().GetAntenna(), *this, *(components_->GetAntenna()));
}
