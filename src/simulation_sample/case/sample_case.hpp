/**
 * @file sample_case.hpp
 * @brief Example of user defined simulation case
 */

#ifndef S2E_SIMULATION_SAMPLE_CASE_SAMPLE_CASE_HPP_
#define S2E_SIMULATION_SAMPLE_CASE_SAMPLE_CASE_HPP_

#include <src/simulation/case/simulation_case.hpp>

#include "../ground_station/sample_ground_station.hpp"
#include "../spacecraft/sample_spacecraft.hpp"

/**
 * @class SampleCase
 * @brief An example of user defined simulation class
 */
class SampleCase : public SimulationCase {
 public:
  /**
   * @fn SampleCase
   * @brief Constructor
   */
  SampleCase(const std::string initialise_base_file);

  /**
   * @fn ~SampleCase
   * @brief Destructor
   */
  virtual ~SampleCase();

  /**
   * @fn GetLogHeader
   * @brief Override function of GetLogHeader
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of GetLogValue
   */
  virtual std::string GetLogValue() const;

 private:
  SampleSpacecraft* sample_spacecraft_;         //!< Instance of spacecraft
  SampleSpacecraft* sample_spacecraft_sub_;     //!< Instance of spacecraft
  SampleGroundStation* sample_ground_station_;  //!< Instance of ground station
  RelativeInformation relative_information_;    //!< Relative information

  /**
   * @fn InitializeTargetObjects
   * @brief Override function of InitializeTargetObjects in SimulationCase
   */
  void InitializeTargetObjects();

  /**
   * @fn UpdateTargetObjects
   * @brief Override function of Main in SimulationCase
   */
  void UpdateTargetObjects();
};

#endif  // S2E_SIMULATION_SAMPLE_CASE_SAMPLE_CASE_HPP_
