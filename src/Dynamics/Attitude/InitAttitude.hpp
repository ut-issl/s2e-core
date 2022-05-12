#pragma once

#include "Attitude.h"
#include "AttitudeRK4.h"
#include "ControlledAttitude.h"

Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* celes_info, const double step_sec,
                       const Matrix<3, 3> inertia_tensor, const int sat_id);
