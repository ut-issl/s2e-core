/**
 * @file sample_ground_station_components.hpp
 * @brief An example of ground station related components list
 */

#ifndef S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_H_
#define S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_H_

#include <components/communication/initialize_antenna.hpp>
#include <components/communication/initialize_ground_station_calculator.hpp>

/**
 * @class SampleGSComponents
 * @brief An example of ground station related components list class
 */
class SampleGSComponents {
 public:
  /**
   * @fn SampleGSComponents
   * @brief Constructor
   */
  SampleGSComponents(const SimulationConfig* config);
  /**
   * @fn ~SampleGSComponents
   * @brief Destructor
   */
  ~SampleGSComponents();
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
  inline Antenna* GetAntenna() const { return antenna_; }
  /**
   * @fn GetGsCalculator
   * @brief Return ground station calculator
   */
  inline GScalculator* GetGsCalculator() const { return gs_calculator_; }

 private:
  Antenna* antenna_;                //!< Antenna on ground station
  GScalculator* gs_calculator_;     //!< Ground station calculation algorithm
  const SimulationConfig* config_;  //!< Simulation setting
};

#endif  // S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_COMPONENTS_H_
