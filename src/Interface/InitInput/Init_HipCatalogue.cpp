#include "../../Environment/Global/HipparcosCatalogue.h"
#include "Initialize.h"

HipparcosCatalogue *InitHipCatalogue(std::string file_name) {
  IniAccess ini_file(file_name);
  char *section = "HIPPARCOS_CATALOGUE";

  double max_magnitude = ini_file.ReadDouble(section, "max_magnitude");
  std::string catalogue_path = ini_file.ReadString(section, "catalogue_path");

  HipparcosCatalogue *hip_catalogue;
  hip_catalogue = new HipparcosCatalogue(max_magnitude, catalogue_path);
  hip_catalogue->IsCalcEnabled = ini_file.ReadEnable(section, CALC_LABEL);
  hip_catalogue->IsLogEnabled = ini_file.ReadEnable(section, LOG_LABEL);
  hip_catalogue->ReadContents(catalogue_path, ',');

  return hip_catalogue;
}
