/**
 * @file sample_ground_station.cpp
 * @brief An example of user defined ground station class
 */

#include "sample_ground_station.hpp"

#include "sample_ground_station_components.hpp"

SampleGS::SampleGS(const SimulationConfig* configuration, const unsigned int ground_station_id) : GroundStation(configuration, ground_station_id) {
  Initialize(configuration);
}

SampleGS::~SampleGS() { delete components_; }

void SampleGS::Initialize(const SimulationConfig* configuration) { components_ = new SampleGsComponents(configuration); }

void SampleGS::LogSetup(Logger& logger) {
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGS::Update(const CelestialRotation& celestial_rotation, const SampleSat& spacecraft) {
  GroundStation::Update(celestial_rotation, spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, spacecraft.GetInstalledComponents().GetAntenna(), *this, *(components_->GetAntenna()));
}
