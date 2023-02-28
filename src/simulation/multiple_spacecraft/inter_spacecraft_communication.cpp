/**
 * @file inter_spacecraft_communication.cpp
 * @brief Base class of inter satellite communication
 */

#include "inter_spacecraft_communication.hpp"

#include <library/utilities/macros.hpp>

InterSpacecraftCommunication::InterSpacecraftCommunication(const SimulationConfiguration* simulation_configuration) {
  UNUSED(simulation_configuration);
}

InterSpacecraftCommunication::~InterSpacecraftCommunication() {}
