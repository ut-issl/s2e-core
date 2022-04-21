#pragma once

#include "Orbit.h"

class RelativeInformation;

Orbit* InitOrbit(const CelestialInformation* celes_info, std::string ini_path, double stepSec, double current_jd, double gravity_constant,
                 std::string section = "ORBIT", RelativeInformation* rel_info = (RelativeInformation*)nullptr);
