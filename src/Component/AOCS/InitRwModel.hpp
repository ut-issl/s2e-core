#pragma once

#include <Component/AOCS/RWModel.h>

RWModel InitRWModel(ClockGenerator* clock_gen, int actuator_id, std::string file_name, double prop_step, double compo_update_step);
RWModel InitRWModel(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, std::string file_name, double prop_step,
                    double compo_update_step);
