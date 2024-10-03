/**
 * @file inter_spacecraft_communication.cpp
 * @brief Base class of inter satellite communication
 */

#include "inter_spacecraft_communication.hpp"

#include <utilities/macros.hpp>

namespace s2e::simulation {

InterSpacecraftCommunication::InterSpacecraftCommunication(const SimulationConfiguration* simulation_configuration) {
  UNUSED(simulation_configuration);
}

InterSpacecraftCommunication::~InterSpacecraftCommunication() {}

}  // namespace s2e::simulation
