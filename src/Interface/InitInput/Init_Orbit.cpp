#include <cassert>

#include "../../Dynamics/Orbit/EarthCenteredOrbit.h"
#include "../../Dynamics/Orbit/RelativeOrbit.h"
#include "../../Dynamics/Orbit/SimpleCircularOrbit.h"
#include "../../Environment/Global/SimTime.h"
#include "../../Library/RelativeOrbit/RelativeOrbitModels.h"
#include "../../RelativeInformation/RelativeInformation.h"
#include "Initialize.h"

class EarthCenteredOrbit;
class SimpleCircularOrbit;
class RelativeOrbit;

Orbit *InitOrbit(const CelestialInformation *celes_info, std::string ini_path,
                 double stepSec, double current_jd, double gravity_constant,
                 std::string section, RelativeInformation *rel_info) {
  auto conf = IniAccess(ini_path);
  const char *section_ = section.c_str();
  Orbit *orbit;

  int propagate_mode = conf.ReadInt(section_, "propagate_mode");

  // Initialize SGP4 orbit propagator
  if (propagate_mode == 1) {
    char tle1[80], tle2[80];
    conf.ReadChar(section_, "tle1", 80, tle1);
    conf.ReadChar(section_, "tle2", 80, tle2);

    int wgs = conf.ReadInt(section_, "wgs");
    orbit = new EarthCenteredOrbit(celes_info, tle1, tle2, wgs, current_jd);
  } else if (propagate_mode ==
             2) // initialize orbit for relative dynamics of formation flying
  {
    int wgs = conf.ReadInt(section_, "wgs");
    RelativeOrbit::RelativeOrbitUpdateMethod update_method =
        (RelativeOrbit::RelativeOrbitUpdateMethod)(
            conf.ReadInt(section_, "relative_orbit_update_method"));
    RelativeOrbitModel relative_dynamics_model_type =
        (RelativeOrbitModel)(conf.ReadInt(section_,
                                          "relative_dynamics_model_type"));
    STMModel stm_model_type =
        (STMModel)(conf.ReadInt(section_, "stm_model_type"));

    Vector<3> init_relative_position_lvlh;
    conf.ReadVector<3>(section_, "init_relative_position_lvlh",
                       init_relative_position_lvlh);
    Vector<3> init_relative_velocity_lvlh;
    conf.ReadVector<3>(section_, "init_relative_velocity_lvlh",
                       init_relative_velocity_lvlh);

    // There is a possibility that the orbit of the reference sat is not
    // initialized when RelativeOrbit initialization is called To ensure that
    // the orbit of the reference sat is initialized, create temporary initial
    // orbit of the reference sat
    int reference_sat_id = conf.ReadInt(section_, "reference_sat_id");

    orbit = new RelativeOrbit(
        gravity_constant, stepSec, wgs, current_jd, reference_sat_id,
        init_relative_position_lvlh, init_relative_velocity_lvlh, update_method,
        relative_dynamics_model_type, stm_model_type, rel_info);
  } else // initialize orbit for RK4 propagation
  {
    int wgs = conf.ReadInt(section_, "wgs");
    Vector<3> init_pos;
    conf.ReadVector<3>(section_, "init_position", init_pos);
    Vector<3> init_veloc;
    conf.ReadVector<3>(section_, "init_velocity", init_veloc);

    orbit = new SimpleCircularOrbit(celes_info, gravity_constant, stepSec, wgs,
                                    init_pos, init_veloc, current_jd);
  }

  orbit->IsCalcEnabled = conf.ReadEnable(section_, CALC_LABEL);
  orbit->IsLogEnabled = conf.ReadEnable(section_, LOG_LABEL);
  return orbit;
}
