#pragma once

#include <Component/AOCS/SunSensor.h>

SunSensor InitSunSensor(ClockGenerator* clock_gen, int sensor_id, const std::string fname, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info);
SunSensor InitSunSensor(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info);
