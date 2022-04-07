#include <Dynamics/Orbit/EarthCenteredOrbit.h>
#include <Dynamics/Orbit/EnckeOrbitPropagation.h>
#include <Dynamics/Orbit/KeplerOrbitPropagation.h>
#include <Dynamics/Orbit/Orbit.h>
#include <Dynamics/Orbit/RelativeOrbit.h>
#include <Dynamics/Orbit/SimpleCircularOrbit.h>
#include <Environment/Global/SimTime.h>
#include <Library/RelativeOrbit/RelativeOrbitModels.h>
#include <RelativeInformation/RelativeInformation.h>

#include <cassert>

#include "Initialize.h"

class EarthCenteredOrbit;
class SimpleCircularOrbit;
class RelativeOrbit;
class KeplerOrbit;

Orbit* InitOrbit(const CelestialInformation* celes_info, std::string ini_path, double stepSec, double current_jd, double gravity_constant,
                 std::string section, RelativeInformation* rel_info) {
  auto conf = IniAccess(ini_path);
  const char* section_ = section.c_str();
  Orbit* orbit;

  std::string propagate_mode = conf.ReadString(section_, "propagate_mode");
  int wgs = conf.ReadInt(section_, "wgs");

  if (propagate_mode == "RK4")  // initialize RK4 orbit propagator
  {
    Vector<3> init_pos;
    conf.ReadVector<3>(section_, "init_position", init_pos);
    Vector<3> init_veloc;
    conf.ReadVector<3>(section_, "init_velocity", init_veloc);

    orbit = new SimpleCircularOrbit(celes_info, gravity_constant, stepSec, wgs, init_pos, init_veloc, current_jd);
  } else if (propagate_mode == "SGP4") {  // Initialize SGP4 orbit propagator
    char tle1[80], tle2[80];
    conf.ReadChar(section_, "tle1", 80, tle1);
    conf.ReadChar(section_, "tle2", 80, tle2);

    orbit = new EarthCenteredOrbit(celes_info, tle1, tle2, wgs, current_jd);
  } else if (propagate_mode == "RELATIVE")  // initialize orbit for relative dynamics of formation flying
  {
    RelativeOrbit::RelativeOrbitUpdateMethod update_method =
        (RelativeOrbit::RelativeOrbitUpdateMethod)(conf.ReadInt(section_, "relative_orbit_update_method"));
    RelativeOrbitModel relative_dynamics_model_type = (RelativeOrbitModel)(conf.ReadInt(section_, "relative_dynamics_model_type"));
    STMModel stm_model_type = (STMModel)(conf.ReadInt(section_, "stm_model_type"));

    Vector<3> init_relative_position_lvlh;
    conf.ReadVector<3>(section_, "init_relative_position_lvlh", init_relative_position_lvlh);
    Vector<3> init_relative_velocity_lvlh;
    conf.ReadVector<3>(section_, "init_relative_velocity_lvlh", init_relative_velocity_lvlh);

    // There is a possibility that the orbit of the reference sat is not
    // initialized when RelativeOrbit initialization is called To ensure that
    // the orbit of the reference sat is initialized, create temporary initial
    // orbit of the reference sat
    int reference_sat_id = conf.ReadInt(section_, "reference_sat_id");

    orbit = new RelativeOrbit(gravity_constant, stepSec, wgs, current_jd, reference_sat_id, init_relative_position_lvlh, init_relative_velocity_lvlh,
                              update_method, relative_dynamics_model_type, stm_model_type, rel_info);
  } else if (propagate_mode == "KEPLER") {
    int init_mode_kepler = conf.ReadInt(section_, "init_mode_kepler");
    double mu_m3_s2 = gravity_constant;
    OrbitalElements oe;
    if (init_mode_kepler == 0)  // initialize with position and velocity
    {
      Vector<3> init_pos_m;
      conf.ReadVector<3>(section_, "init_position", init_pos_m);
      Vector<3> init_vel_m_s;
      conf.ReadVector<3>(section_, "init_velocity", init_vel_m_s);
      oe = OrbitalElements(mu_m3_s2, current_jd, init_pos_m, init_vel_m_s);
    } else  // initialize with orbital elements
    {
      double semi_major_axis_m = conf.ReadDouble(section_, "semi_major_axis_m");
      double eccentricity = conf.ReadDouble(section_, "eccentricity");
      double inclination_rad = conf.ReadDouble(section_, "inclination_rad");
      double raan_rad = conf.ReadDouble(section_, "raan_rad");
      double arg_perigee_rad = conf.ReadDouble(section_, "arg_perigee_rad");
      double epoch_jday = conf.ReadDouble(section_, "epoch_jday");
      oe = OrbitalElements(epoch_jday, semi_major_axis_m, eccentricity, inclination_rad, raan_rad, arg_perigee_rad);
    }
    KeplerOrbit kepler_orbit(mu_m3_s2, current_jd, oe);
    orbit = new KeplerOrbitPropagation(current_jd, kepler_orbit, wgs);
  } else if (propagate_mode == "ENCKE") {
    Vector<3> init_pos_m;
    conf.ReadVector<3>(section_, "init_position", init_pos_m);
    Vector<3> init_vel_m_s;
    conf.ReadVector<3>(section_, "init_velocity", init_vel_m_s);
    double error_tolerance = conf.ReadDouble(section_, "error_tolerance");
    orbit = new EnckeOrbitPropagation(gravity_constant, stepSec, current_jd, init_pos_m, init_vel_m_s, error_tolerance, wgs);
  } else {
    std::cerr << "ERROR: orbit propagation mode: " << propagate_mode << " is not defined!" << std::endl;
  }

  orbit->IsCalcEnabled = conf.ReadEnable(section_, CALC_LABEL);
  orbit->IsLogEnabled = conf.ReadEnable(section_, LOG_LABEL);
  return orbit;
}
