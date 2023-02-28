/*
 * @file initialize_telescope.cpp
 * @brief Initialize function of Telescope
 */

#include "initialize_telescope.hpp"

#include <string.h>

#include <library/math/constants.hpp>

#include "library/initialize/initialize_file_access.hpp"

using namespace std;

Telescope InitTelescope(ClockGenerator* clock_generator, int sensor_id, const string file_name, const Attitude* attitude,
                        const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information) {
  using libra::pi;

  IniAccess Telescope_conf(file_name);
  const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id));
  const char* cs = st_sensor_id.data();

  char TelescopeSection[30] = "TELESCOPE_";
#ifdef WIN32
  strcat_s(TelescopeSection, cs);
#else
  strcat(TelescopeSection, cs);
#endif

  Quaternion quaternion_b2c;
  Telescope_conf.ReadQuaternion(TelescopeSection, "quaternion_b2c", quaternion_b2c);

  double sun_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * pi / 180;  // deg to rad
  double earth_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * pi / 180;  // deg to rad
  double moon_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * pi / 180;  // deg to rad

  int x_number_of_pix = Telescope_conf.ReadInt(TelescopeSection, "x_number_of_pixel");
  int y_number_of_pix = Telescope_conf.ReadInt(TelescopeSection, "y_number_of_pixel");

  double x_fov_per_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "x_fov_deg_per_pixel");
  double x_fov_per_pix_rad = x_fov_per_pix_deg * pi / 180;  // deg to rad
  double y_fov_per_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "y_fov_deg_per_pixel");
  double y_fov_per_pix_rad = y_fov_per_pix_deg * pi / 180;  // deg to rad

  int number_of_logged_stars = Telescope_conf.ReadInt(TelescopeSection, "number_of_stars_for_log");

  Telescope telescope(clock_generator, quaternion_b2c, sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, x_number_of_pix,
                      y_number_of_pix, x_fov_per_pix_rad, y_fov_per_pix_rad, number_of_logged_stars, attitude, hipparcos,
                      local_celestial_information);
  return telescope;
}
