/*
 * @file InitPCU_InitialStudy.hpp
 * @brief Initialize function of PCU_InitialStudy
 */
#pragma once

#include <components/power/PCU_InitialStudy.h>

/*
 * @fn InitPCU_InitialStudy
 * @brief Initialize function of BAT
 * @param [in] clock_gen: Clock generator
 * @param [in] pcu_id: Power Control Unit ID
 * @param [in] fname: Path to initialize file
 * @param [in] sap: Solar Array Panel infomation
 * @param [in] bat: Battery information
 * @param [in] compo_step_time: Component step time [sec]
 */
PCU_InitialStudy InitPCU_InitialStudy(ClockGenerator* clock_gen, int pcu_id, const std::string fname, const std::vector<SAP*> saps, BAT* bat,
                                      double compo_step_time);
