#pragma once

#include "ForceGenerator.hpp"

ForceGenerator InitializeForceGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics);
