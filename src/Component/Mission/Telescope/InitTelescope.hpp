#pragma once

#include <Component/Mission/Telescope/Telescope.h>

Telescope InitTelescope(ClockGenerator* clock_gen, int sensor_id, const std::string fname, const Attitude* attitude, const HipparcosCatalogue* hipp,
                        const LocalCelestialInformation* local_celes_info);
