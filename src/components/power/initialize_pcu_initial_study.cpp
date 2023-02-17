/*
 * @file initialize_pcu_initial_study.cpp
 * @brief Initialize function of PCU_InitialStudy
 */

#define _CRT_SECURE_NO_WARNINGS
#include "initialize_pcu_initial_study.hpp"

#include <string>
#include <vector>

#include "library/initialize/initialize_file_access.hpp"

PCU_InitialStudy InitPCU_InitialStudy(ClockGenerator* clock_gen, int pcu_id, const std::string fname, const std::vector<SAP*> saps, BAT* bat,
                                      double compo_step_time) {
  IniAccess pcu_conf(fname);

  const std::string st_pcu_id = std::to_string(pcu_id);
  const char* cs = st_pcu_id.data();

  char Section[30] = "PCU_INITIAL_STUDY_";
  strcat(Section, cs);

  int prescaler = pcu_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  PCU_InitialStudy pcu(prescaler, clock_gen, saps, bat, compo_step_time);

  return pcu;
}
