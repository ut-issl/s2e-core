#pragma once

#include <Component/AOCS/Gyro.h>

Gyro InitGyro(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics);
Gyro InitGyro(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
              const Dynamics* dynamics);
