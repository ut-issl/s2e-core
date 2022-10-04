#pragma once

#include <Component/Power/BAT.h>

BAT InitBAT(ClockGenerator* clock_gen, int bat_id, const std::string fname, double compo_step_time);
