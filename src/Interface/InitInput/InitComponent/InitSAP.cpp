#define _CRT_SECURE_NO_WARNINGS
#include <string.h>
#include "../Initialize.h"
#include <Component/Power/SAP.h>

SAP InitSAP(ClockGenerator* clock_gen, int sap_id, const std::string fname, const SRPEnvironment* srp) {

  IniAccess sap_conf(fname);

  const std::string st_sap_id = std::to_string(static_cast<long long>(sap_id));
  const char *cs = st_sap_id.data();

  char Section[30] = "SAP";
  strcat(Section, cs);

  int number_of_series;
  number_of_series = sap_conf.ReadInt(Section, "number_of_series");

  int number_of_parallel;
  number_of_parallel = sap_conf.ReadInt(Section, "number_of_parallel");

  double cell_area;
  cell_area = sap_conf.ReadDouble(Section, "cell_area");

  Vector<3> normal_vector;
  sap_conf.ReadVector(Section, "normal_vector", normal_vector);

  double cell_efficiency;
  cell_efficiency = sap_conf.ReadDouble(Section, "cell_efficiency");

  double transmission_efficiency;
  transmission_efficiency = sap_conf.ReadDouble(Section, "transmission_efficiency");

  SAP sap(clock_gen, sap_id, number_of_series, number_of_parallel, cell_area, normal_vector, cell_efficiency, transmission_efficiency, srp);

  return sap;
}
