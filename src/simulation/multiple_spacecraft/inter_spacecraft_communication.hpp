/**
 * @file inter_spacecraft_communication.h
 * @brief Base class of inter satellite communication
 */

#ifndef S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
#define S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_

#include "../simulation_configuration.hpp"

/**
 * @class InterSatComm
 * @brief Base class of inter satellite communication
 */
class InterSatComm {
 public:
  /**
   * @fn InterSatComm
   * @brief Constructor
   */
  InterSatComm(const SimulationConfig* simulation_configuration);
  /**
   * @fn ~InterSatComm
   * @brief Destructor
   */
  ~InterSatComm();

 private:
};

#endif  // S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
