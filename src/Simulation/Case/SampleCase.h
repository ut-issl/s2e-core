/**
 * @file SampleCase.h
 * @brief Example of user defined simulation case
 */

#pragma once

#include "../GroundStation/SampleGroundStation/SampleGS.h"
#include "../Spacecraft/SampleSpacecraft/SampleSat.h"
#include "./SimulationCase.h"

/**
 * @class SampleCase
 * @brief An example of user side spacecraft class
 */
class SampleCase : public SimulationCase {
 public:
  /**
   * @fn SampleCase
   * @brief Constructor
   */
  SampleCase(std::string ini_base);

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
  SampleSat* sample_sat_;  //!< Instance of spacecraft
  SampleGS* sample_gs_;    //!< Instance of ground station
};
