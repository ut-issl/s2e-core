#pragma once

#include <Component/Power/PCU_InitialStudy.h>

PCU_InitialStudy InitPCU_InitialStudy(ClockGenerator* clock_gen, int pcu_id, const std::string fname, const std::vector<SAP*> saps, BAT* bat, double compo_step_time);
