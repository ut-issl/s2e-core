#pragma once

#include "Temperature.h"
class Temperature;

Temperature* InitTemperature(const std::string ini_path, const double rk_prop_step_sec);
