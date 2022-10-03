#define _CRT_SECURE_NO_WARNINGS
#include "InitPcu_InitialStudy.hpp"

#include <string>
#include <vector>

#include "Interface/InitInput/IniAccess.h"

PCU_InitialStudy InitPCU_InitialStudy(ClockGenerator* clock_gen, int pcu_id, const std::string fname, const std::vector<SAP*> saps, BAT* bat, double compo_step_time) {
  IniAccess pcu_conf(fname);

  const std::string st_pcu_id = std::to_string(static_cast<long long>(pcu_id));
  const char* cs = st_pcu_id.data();

  char Section[30] = "PCU";
  strcat(Section, cs);

  int prescaler = pcu_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  PCU_InitialStudy pcu(prescaler, clock_gen, saps, bat, compo_step_time);

  return pcu;
}
