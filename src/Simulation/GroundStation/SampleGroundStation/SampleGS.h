/**
 * @file SampleGS.h
 * @brief An example of user defined ground station class
 */

#pragma once

#include <Component/CommGS/GScalculator.h>
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>

#include <Component/CommGS/Antenna.hpp>

#include "../GroundStation.h"

class SampleGSComponents;

/**
 * @class SampleGS
 * @brief An example of user defined ground station class
 */
class SampleGS : public GroundStation {
 public:
  /**
   * @fn SampleGS
   * @brief Constructor
   */
  SampleGS(SimulationConfig* config, int gs_id);
  /**
   * @fn ~SampleGS
   * @brief Deconstructor
   */
  ~SampleGS();

  /**
   * @fn Initialize
   * @brief Override function of Initialize in GroundStation class
   */
  virtual void Initialize(SimulationConfig* config);
  /**
   * @fn LogSetup
   * @brief Override function of LogSetup in GroundStation class
   */
  virtual void LogSetup(Logger& logger);
  /**
   * @fn Update
   * @brief Override function of Update in GroundStation class
   */
  virtual void Update(const Spacecraft& spacecraft, const GlobalEnvironment& global_env, const Antenna& sc_ant, const SampleGS& sample_gs);

 private:
  SampleGSComponents* components_;  //!< Ground station related components
};
