/**
 * @file inter_spacecraft_communication.cpp
 * @brief Base class of inter satellite communication
 */

#include "inter_spacecraft_communication.hpp"

#include <library/utilities/Macros.hpp>

InterSatComm::InterSatComm(const SimulationConfig* sim_config) { UNUSED(sim_config); }

InterSatComm::~InterSatComm() {}
