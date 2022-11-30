/**
 * @file SampleGSComponents.h
 * @brief An example of ground station related components list
 */

#pragma once

#include <Component/CommGS/InitAntenna.hpp>
#include <Component/CommGS/InitGsCalculator.hpp>

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
   * @brief Deconstructor
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
