/**
 * @file sample_ground_station.cpp
 * @brief An example of user defined ground station class
 */

#include "sample_ground_station.hpp"

#include "sample_ground_station_components.hpp"

SampleGroundStation::SampleGroundStation(const SimulationConfiguration* configuration, const unsigned int ground_station_id)
    : GroundStation(configuration, ground_station_id) {
  components_ = new SampleGsComponents(configuration);
}

SampleGroundStation::~SampleGroundStation() { delete components_; }

void SampleGroundStation::LogSetup(Logger& logger) {
  GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGroundStation::Update(const CelestialRotation& celestial_rotation, const SampleSpacecraft& spacecraft) {
  GroundStation::Update(celestial_rotation, spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, spacecraft.GetInstalledComponents().GetAntenna(), *this, *(components_->GetAntenna()));
}
