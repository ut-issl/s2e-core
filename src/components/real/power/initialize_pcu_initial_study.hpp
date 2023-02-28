/*
 * @file initialize_pcu_initial_study.hpp
 * @brief Initialize function of PCU_InitialStudy
 */

#ifndef S2E_COMPONENTS_REAL_POWER_INITIALIZE_PCU_INITIAL_STUDY_HPP_
#define S2E_COMPONENTS_REAL_POWER_INITIALIZE_PCU_INITIAL_STUDY_HPP_

#include <components/real/power/pcu_initial_study.hpp>

/*
 * @fn InitPCU_InitialStudy
 * @brief Initialize function of Battery
 * @param [in] clock_generator: Clock generator
 * @param [in] pcu_id: Power Control Unit ID
 * @param [in] file_name: Path to initialize file
 * @param [in] sap: Solar Array Panel infomation
 * @param [in] battery: Battery information
 * @param [in] component_step_time_s: Component step time [sec]
 */
PCU_InitialStudy InitPCU_InitialStudy(ClockGenerator* clock_generator, int pcu_id, const std::string file_name, const std::vector<SAP*> saps,
                                      Battery* battery, double component_step_time_s);

#endif  // S2E_COMPONENTS_REAL_POWER_INITIALIZE_PCU_INITIAL_STUDY_HPP_
