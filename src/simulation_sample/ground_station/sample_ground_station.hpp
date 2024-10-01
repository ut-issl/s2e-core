/**
 * @file sample_ground_station.hpp
 * @brief An example of user defined ground station class
 */

#ifndef S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_HPP_
#define S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_HPP_

#include <components/real/communication/antenna.hpp>
#include <components/real/communication/ground_station_calculator.hpp>
#include <dynamics/dynamics.hpp>
#include <environment/global/global_environment.hpp>
#include <simulation/ground_station/ground_station.hpp>

#include "../spacecraft/sample_spacecraft.hpp"

namespace s2e::sample {

class SampleGsComponents;

/**
 * @class SampleGroundStation
 * @brief An example of user defined ground station class
 */
class SampleGroundStation : public simulation::GroundStation {
 public:
  /**
   * @fn SampleGroundStation
   * @brief Constructor
   */
  SampleGroundStation(const simulation::SimulationConfiguration* configuration, const unsigned int ground_station_id);
  /**
   * @fn ~SampleGroundStation
   * @brief Destructor
   */
  ~SampleGroundStation();

  /**
   * @fn LogSetup
   * @brief Override function of LogSetup in GroundStation class
   */
  virtual void LogSetup(logger::Logger& logger);
  /**
   * @fn Update
   * @brief Override function of Update in GroundStation class
   */
  virtual void Update(const environment::EarthRotation& celestial_rotation, const SampleSpacecraft& spacecraft);

 private:
  using GroundStation::Update;
  SampleGsComponents* components_;  //!< Ground station related components
};

}  // namespace s2e::sample

#endif  // S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_HPP_
