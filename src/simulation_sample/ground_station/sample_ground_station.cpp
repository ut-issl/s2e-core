/**
 * @file sample_ground_station.cpp
 * @brief An example of user defined ground station class
 */

#include "sample_ground_station.hpp"

#include "sample_ground_station_components.hpp"

namespace s2e::sample {

SampleGroundStation::SampleGroundStation(const simulation::SimulationConfiguration* configuration, const unsigned int ground_station_id)
    : simulation::GroundStation(configuration, ground_station_id) {
  components_ = new SampleGsComponents(configuration);
}

SampleGroundStation::~SampleGroundStation() { delete components_; }

void SampleGroundStation::LogSetup(logger::Logger& logger) {
  simulation::GroundStation::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleGroundStation::Update(const environment::EarthRotation& celestial_rotation, const SampleSpacecraft& spacecraft) {
  simulation::GroundStation::Update(celestial_rotation, spacecraft);
  components_->GetGsCalculator()->Update(spacecraft, spacecraft.GetInstalledComponents().GetAntenna(), *this, *(components_->GetAntenna()));
}

}  // namespace s2e::sample
