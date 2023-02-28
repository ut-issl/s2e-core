/**
 * @file inter_spacecraft_communication.h
 * @brief Base class of inter satellite communication
 */

#ifndef S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
#define S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_

#include "../simulation_configuration.hpp"

/**
 * @class InterSpacecraftCommunication
 * @brief Base class of inter satellite communication
 */
class InterSpacecraftCommunication {
 public:
  /**
   * @fn InterSpacecraftCommunication
   * @brief Constructor
   */
  InterSpacecraftCommunication(const SimulationConfig* simulation_configuration);
  /**
   * @fn ~InterSpacecraftCommunication
   * @brief Destructor
   */
  ~InterSpacecraftCommunication();

 private:
};

#endif  // S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
