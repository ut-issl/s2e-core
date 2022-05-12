#pragma once

#include <Component/AOCS/STT.h>

STT InitSTT(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics,
            const LocalEnvironment* local_env);
STT InitSTT(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
            const Dynamics* dynamics, const LocalEnvironment* local_env);
