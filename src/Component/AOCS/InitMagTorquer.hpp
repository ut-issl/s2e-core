#pragma once

#include <Component/AOCS/MagTorquer.h>

MagTorquer InitMagTorquer(ClockGenerator* clock_gen, int actuator_id, const std::string fname, double compo_step_time, const MagEnvironment* mag_env);
MagTorquer InitMagTorquer(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, const std::string fname, double compo_step_time,
                          const MagEnvironment* mag_env);
