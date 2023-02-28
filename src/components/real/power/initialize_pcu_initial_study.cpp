/*
 * @file initialize_pcu_initial_study.cpp
 * @brief Initialize function of PcuInitialStudy
 */

#define _CRT_SECURE_NO_WARNINGS
#include "initialize_pcu_initial_study.hpp"

#include <string>
#include <vector>

#include "library/initialize/initialize_file_access.hpp"

PcuInitialStudy InitPCU_InitialStudy(ClockGenerator* clock_generator, int pcu_id, const std::string file_name, const std::vector<SAP*> saps,
                                     Battery* battery, double component_step_time_s) {
  IniAccess pcu_conf(file_name);

  const std::string st_pcu_id = std::to_string(pcu_id);
  const char* cs = st_pcu_id.data();

  char Section[30] = "PCU_INITIAL_STUDY_";
  strcat(Section, cs);

  int prescaler = pcu_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  PcuInitialStudy pcu(prescaler, clock_generator, saps, battery, component_step_time_s);

  return pcu;
}
