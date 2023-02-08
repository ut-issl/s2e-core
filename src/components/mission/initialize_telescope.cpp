/*
 * @file initialize_telescope.cpp
 * @brief Initialize function of Telescope
 */

#include "initialize_telescope.hpp"

#include <string.h>

#include <library/math/Constant.hpp>

#include "interface/initialize/initialize_file_access.hpp"

using namespace std;

Telescope InitTelescope(ClockGenerator* clock_gen, int sensor_id, const string fname, const Attitude* attitude, const HipparcosCatalogue* hipp,
                        const LocalCelestialInformation* local_celes_info) {
  using libra::pi;

  IniAccess Telescope_conf(fname);
  const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id));
  const char* cs = st_sensor_id.data();

  char TelescopeSection[30] = "TELESCOPE_";
#ifdef WIN32
  strcat_s(TelescopeSection, cs);
#else
  strcat(TelescopeSection, cs);
#endif

  Quaternion q_b2c;
  Telescope_conf.ReadQuaternion(TelescopeSection, "quaternion_b2c", q_b2c);

  double sun_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * pi / 180;  // deg to rad
  double earth_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * pi / 180;  // deg to rad
  double moon_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * pi / 180;  // deg to rad

  int x_num_of_pix = Telescope_conf.ReadInt(TelescopeSection, "x_number_of_pixel");
  int y_num_of_pix = Telescope_conf.ReadInt(TelescopeSection, "y_number_of_pixel");

  double x_fov_par_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "x_fov_deg_per_pixel");
  double x_fov_par_pix_rad = x_fov_par_pix_deg * pi / 180;  // deg to rad
  double y_fov_par_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "y_fov_deg_per_pixel");
  double y_fov_par_pix_rad = y_fov_par_pix_deg * pi / 180;  // deg to rad

  int num_of_logged_stars = Telescope_conf.ReadInt(TelescopeSection, "number_of_stars_for_log");

  Telescope telescope(clock_gen, q_b2c, sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, x_num_of_pix, y_num_of_pix,
                      x_fov_par_pix_rad, y_fov_par_pix_rad, num_of_logged_stars, attitude, hipp, local_celes_info);
  return telescope;
}
