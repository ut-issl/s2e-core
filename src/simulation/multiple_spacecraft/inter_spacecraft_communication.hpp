/**
 * @file inter_spacecraft_communication.h
 * @brief Base class of inter satellite communication
 */

#ifndef S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
#define S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_

#include "../simulation_configuration.hpp"

namespace s2e::simulation {

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
  InterSpacecraftCommunication(const SimulationConfiguration* simulation_configuration);
  /**
   * @fn ~InterSpacecraftCommunication
   * @brief Destructor
   */
  ~InterSpacecraftCommunication();

 private:
};

} // namespace s2e::simulation

#endif  // S2E_SIMULATION_MULTIPLE_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
