/**
 * @file sample_case.hpp
 * @brief Example of user defined simulation case
 */

#ifndef S2E_SIMULATION_CASE_SAMPLE_CASE_HPP_
#define S2E_SIMULATION_CASE_SAMPLE_CASE_HPP_

#include "../ground_station/sample_ground_station/sample_ground_station.hpp"
#include "../spacecraft/sample_spacecraft/sample_spacecraft.hpp"
#include "./simulation_case.hpp"

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
  SampleCase(std::string initialise_base_file);

  /**
   * @fn ~SampleCase
   * @brief Destructor
   */
  virtual ~SampleCase();

  /**
   * @fn Initialize
   * @brief Override function of Initialize in SimulationCase
   */
  void Initialize();

  /**
   * @fn Main
   * @brief Override function of Main in SimulationCase
   */
  void Main();

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
  SampleGroundStation* sample_ground_station_;  //!< Instance of ground station
};

#endif  // S2E_SIMULATION_CASE_SAMPLE_CASE_HPP_
