/**
 * @file sample_ground_station.h
 * @brief An example of user defined ground station class
 */

#ifndef S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_H_
#define S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_H_

#include <Component/CommGS/GScalculator.h>

#include <Component/CommGS/Antenna.hpp>
#include <dynamics/dynamics.hpp>
#include <environment/global/global_environment.hpp>

#include "../../spacecraft/sample_spacecraft/sample_spacecraft.hpp"
#include "../ground_station.hpp"

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
   * @brief Destructor
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
  virtual void Update(const CelestialRotation& celes_rotation, const SampleSat& spacecraft);

 private:
  SampleGSComponents* components_;  //!< Ground station related components
};

#endif  // S2E_SIMULATION_GROUND_STATION_SAMPLE_GROUND_STATION_SAMPLE_GROUND_STATION_H_
