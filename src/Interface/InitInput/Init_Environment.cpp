#include "Initialize.h"
#include "../../Environment/MagEnvironment.h"
#include "../../Environment/SRPEnvironment.h"
#include "../../Environment/Atmosphere.h"

MagEnvironment InitMagEnvironment(string ini_path)
{
  auto conf = IniAccess(ini_path);
  const char* section = "MAG_ENVIRONMENT";

  string fname = conf.ReadString(section,"coeff_file");
  double mag_rwdev = conf.ReadDouble(section, "mag_rwdev");
  double mag_rwlimit = conf.ReadDouble(section, "mag_rwlimit");
  double mag_wnvar = conf.ReadDouble(section, "mag_wnvar");

  MagEnvironment mag_env(fname, mag_rwdev, mag_rwlimit, mag_wnvar);
  mag_env.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  mag_env.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);


  return mag_env;
}

SRPEnvironment InitSRPEnvironment(string ini_path)
{
  auto conf = IniAccess(ini_path);
  const char* section = "SRP";
  
  SRPEnvironment srp_env;
  srp_env.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  srp_env.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return srp_env;
}

Atmosphere InitAtmosphere(string ini_path)
{
  auto conf = IniAccess(ini_path);
  const char* section = "ATMOSPHERE";

  string model = conf.ReadString(section, "model");
  string table_path = conf.ReadString(section, "nrlmsise00_table_path");
  double rho_stddev = conf.ReadDouble(section, "rho_stddev");
  Atmosphere atmosphere(model, table_path, rho_stddev);
  atmosphere.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  atmosphere.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);
  
  return atmosphere;
}
