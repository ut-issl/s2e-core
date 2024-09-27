/**
 * @file sample_ground_station_components.hpp
 * @brief An example of ground station related components list
 */

#ifndef S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_HPP_
#define S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_HPP_

#include <components/real/communication/antenna.hpp>
#include <components/real/communication/ground_station_calculator.hpp>

namespace s2e::sample {

/**
 * @class SampleGsComponents
 * @brief An example of ground station related components list class
 */
class SampleGsComponents {
 public:
  /**
   * @fn SampleGsComponents
   * @brief Constructor
   */
  SampleGsComponents(const simulation::SimulationConfiguration* configuration);
  /**
   * @fn ~SampleGsComponents
   * @brief Destructor
   */
  ~SampleGsComponents();
  /**
   * @fn CompoLogSetUp
   * @brief Log setup for ground station components
   */
  void CompoLogSetUp(Logger& logger);

  // Getter
  /**
   * @fn GetAntenna
   * @brief Return antenna
   */
  inline components::Antenna* GetAntenna() const { return antenna_; }
  /**
   * @fn GetGsCalculator
   * @brief Return ground station calculator
   */
  inline components::GroundStationCalculator* GetGsCalculator() const { return gs_calculator_; }

 private:
  components::Antenna* antenna_;                              //!< Antenna on ground station
  components::GroundStationCalculator* gs_calculator_;        //!< Ground station calculation algorithm
  const simulation::SimulationConfiguration* configuration_;  //!< Simulation setting
};

} // namespace s2e::sample

#endif  // S2E_SIMULATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_HPP_
