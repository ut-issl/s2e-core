/**
 * @file celestial_information.cpp
 * @brief Class to manage the information related with the celestial bodies
 * @details This class uses SPICE to get the information of celestial bodies
 */

#include "celestial_information.hpp"

#include <SpiceUsr.h>
#include <assert.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <locale>
#include <sstream>

#include "initial_setting_file/initialize_file_access.hpp"
#include "library/logger/log_utility.hpp"

CelestialInformation::CelestialInformation(const std::string inertial_frame_name, const std::string aberration_correction_setting,
                                           const std::string center_body_name, const unsigned int number_of_selected_body, int* selected_body_ids,
                                           const std::vector<std::string> rotation_mode_list)
    : number_of_selected_bodies_(number_of_selected_body),
      selected_body_ids_(selected_body_ids),
      inertial_frame_name_(inertial_frame_name),
      center_body_name_(center_body_name),
      aberration_correction_setting_(aberration_correction_setting),
      rotation_mode_list_(rotation_mode_list) {
  // Initialize list
  unsigned int num_of_state = number_of_selected_bodies_ * 3;
  celestial_body_position_from_center_i_m_ = new double[num_of_state];
  celestial_body_velocity_from_center_i_m_s_ = new double[num_of_state];
  celestial_body_gravity_constant_m3_s2_ = new double[number_of_selected_bodies_];
  celestial_body_mean_radius_m_ = new double[number_of_selected_bodies_];
  celestial_body_planetographic_radii_m_ = new double[num_of_state];

  // Acquisition of gravity constant
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    SpiceInt planet_id = selected_body_ids_[i];
    SpiceInt dim;
    SpiceDouble gravity_constant_km3_s2;
    bodvcd_c(planet_id, "GM", 1, &dim, &gravity_constant_km3_s2);
    // Convert unit [km^3/s^2] to [m^3/s^2]
    celestial_body_gravity_constant_m3_s2_[i] = gravity_constant_km3_s2 * 1E+9;
  }

  // Acquisition of radius
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    SpiceInt planet_id = selected_body_ids_[i];
    SpiceInt dim;
    SpiceDouble radii_km[3];

    bodvcd_c(planet_id, "RADII", 3, &dim, (SpiceDouble*)radii_km);
    for (int j = 0; j < 3; j++) {
      celestial_body_planetographic_radii_m_[i * 3 + j] = radii_km[j] * 1000.0;
    }

    double rx = celestial_body_planetographic_radii_m_[i * 3];
    double ry = celestial_body_planetographic_radii_m_[i * 3 + 1];
    double rz = celestial_body_planetographic_radii_m_[i * 3 + 2];

    celestial_body_mean_radius_m_[i] = pow(rx * ry * rz, 1.0 / 3.0);
  }

  // Initialize rotation
  earth_rotation_ = new EarthRotation(ConvertEarthRotationMode(GetRotationMode("EARTH")));
  moon_rotation_ = new MoonRotation(*this, ConvertMoonRotationMode(GetRotationMode("MOON")));
}

CelestialInformation::CelestialInformation(const CelestialInformation& obj)
    : number_of_selected_bodies_(obj.number_of_selected_bodies_),
      inertial_frame_name_(obj.inertial_frame_name_),
      center_body_name_(obj.center_body_name_),
      aberration_correction_setting_(obj.aberration_correction_setting_) {
  unsigned int num_of_state = number_of_selected_bodies_ * 3;

  selected_body_ids_ = new int[number_of_selected_bodies_];
  celestial_body_position_from_center_i_m_ = new double[num_of_state];
  celestial_body_velocity_from_center_i_m_s_ = new double[num_of_state];
  celestial_body_gravity_constant_m3_s2_ = new double[number_of_selected_bodies_];
  celestial_body_mean_radius_m_ = new double[number_of_selected_bodies_];
  celestial_body_planetographic_radii_m_ = new double[num_of_state];

  size_t size_i = sizeof(int);
  memcpy(selected_body_ids_, obj.selected_body_ids_, size_i * number_of_selected_bodies_);

  size_t size_d = sizeof(double);
  memcpy(celestial_body_position_from_center_i_m_, obj.celestial_body_position_from_center_i_m_, size_d * num_of_state);
  memcpy(celestial_body_velocity_from_center_i_m_s_, obj.celestial_body_velocity_from_center_i_m_s_, size_d * num_of_state);
  memcpy(celestial_body_gravity_constant_m3_s2_, obj.celestial_body_gravity_constant_m3_s2_, size_d * number_of_selected_bodies_);
  memcpy(celestial_body_mean_radius_m_, obj.celestial_body_mean_radius_m_, size_d * number_of_selected_bodies_);
  memcpy(celestial_body_planetographic_radii_m_, obj.celestial_body_planetographic_radii_m_, size_d * num_of_state);
}

CelestialInformation::~CelestialInformation() {
  delete[] celestial_body_position_from_center_i_m_;
  delete[] celestial_body_velocity_from_center_i_m_s_;
  delete[] celestial_body_gravity_constant_m3_s2_;
  delete[] celestial_body_mean_radius_m_;
  delete[] celestial_body_planetographic_radii_m_;
  delete[] selected_body_ids_;
  delete earth_rotation_;
}

void CelestialInformation::UpdateAllObjectsInformation(const SimulationTime& simulation_time) {
  // Update celestial body orbit
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    SpiceInt planet_id = selected_body_ids_[i];

    // Acquisition of body name from id
    SpiceBoolean found;
    const int kMaxNameLength = 100;
    char name_buffer[kMaxNameLength];
    bodc2n_c(planet_id, kMaxNameLength, name_buffer, (SpiceBoolean*)&found);

    // Acquisition of position and velocity
    SpiceDouble orbit_buffer_km[6];
    GetPlanetOrbit(name_buffer, simulation_time.GetCurrentEphemerisTime(), (SpiceDouble*)orbit_buffer_km);
    // Convert unit [km], [km/s] to [m], [m/s]
    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_center_i_m_[i * 3 + j] = orbit_buffer_km[j] * 1000.0;
      celestial_body_velocity_from_center_i_m_s_[i * 3 + j] = orbit_buffer_km[j + 3] * 1000.0;
    }
  }

  // Update earth rotation
  earth_rotation_->Update(simulation_time.GetCurrentTime_jd());
  // Update moon rotation
  moon_rotation_->Update(simulation_time);
}

int CelestialInformation::CalcBodyIdFromName(const char* body_name) const {
  int index = 0;
  SpiceInt planet_id;
  SpiceBoolean found;

  // Acquisition of ID from body name
  bodn2c_c(body_name, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    if (selected_body_ids_[i] == planet_id) {
      index = i;
      break;
    }
  }
  return index;
}

std::string CelestialInformation::GetLogHeader() const {
  SpiceBoolean found;
  const int kMaxNameLength = 100;
  char name_buffer[kMaxNameLength];
  std::string str_tmp = "";
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    SpiceInt planet_id = selected_body_ids_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, kMaxNameLength, name_buffer, (SpiceBoolean*)&found);
    std::string name = name_buffer;

    std::locale loc = std::locale::classic();
    std::transform(name.begin(), name.end(), name.begin(), [loc](char c) { return std::tolower(c, loc); });

    std::string body_pos = name + "_position";
    std::string body_vel = name + "_velocity";

    str_tmp += WriteVector(body_pos, "i", "m", 3);
    str_tmp += WriteVector(body_vel, "i", "m/s", 3);
  }
  return str_tmp;
}

std::string CelestialInformation::GetLogValue() const {
  std::string str_tmp = "";
  for (unsigned int i = 0; i < number_of_selected_bodies_; i++) {
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celestial_body_position_from_center_i_m_[i * 3 + j]);
    }
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celestial_body_velocity_from_center_i_m_s_[i * 3 + j]);
    }
  }
  return str_tmp;
}

void CelestialInformation::GetPlanetOrbit(const char* planet_name, const double et, double orbit[6]) {
  // Add `BARYCENTER` if needed
  std::string planet_name_string = planet_name;
  if (strcmp(planet_name, "MARS") == 0 || strcmp(planet_name, "JUPITER") == 0 || strcmp(planet_name, "SATURN") == 0 ||
      strcmp(planet_name, "URANUS") == 0 || strcmp(planet_name, "NEPTUNE") == 0 || strcmp(planet_name, "PLUTO") == 0) {
    planet_name_string += "_BARYCENTER";
  }

  // Get orbit
  SpiceDouble lt;
  spkezr_c((ConstSpiceChar*)planet_name_string.c_str(), (SpiceDouble)et, (ConstSpiceChar*)inertial_frame_name_.c_str(),
           (ConstSpiceChar*)aberration_correction_setting_.c_str(), (ConstSpiceChar*)center_body_name_.c_str(), (SpiceDouble*)orbit,
           (SpiceDouble*)&lt);
  return;
}

CelestialInformation* InitCelestialInformation(std::string file_name) {
  IniAccess ini_file(file_name);
  const char* section = "CELESTIAL_INFORMATION";
  const char* furnsh_section = "CSPICE_KERNELS";

  // Read SPICE setting
  std::string inertial_frame = ini_file.ReadString(section, "inertial_frame");
  std::string aber_cor = ini_file.ReadString(section, "aberration_correction");
  std::string center_obj = ini_file.ReadString(section, "center_object");

  // SPICE Furnsh
  std::vector<std::string> keywords = {"tls", "tpc1", "tpc2", "tpc3", "bsp"};
  for (size_t i = 0; i < keywords.size(); i++) {
    std::string fname = ini_file.ReadString(furnsh_section, keywords[i].c_str());
    furnsh_c(fname.c_str());
  }

  // Initialize celestial body list
  const int num_of_selected_body = ini_file.ReadInt(section, "number_of_selected_body");
  int* selected_body = new int[num_of_selected_body];
  for (int i = 0; i < num_of_selected_body; i++) {
    // Convert body name to SPICE ID
    std::string selected_body_i = "selected_body_name(" + std::to_string(i) + ")";
    char selected_body_temp[30];
    ini_file.ReadChar(section, selected_body_i.c_str(), 30, selected_body_temp);
    SpiceInt planet_id;
    SpiceBoolean found;
    bodn2c_c(selected_body_temp, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);

    // If the object specified in the ini file is not found, exit the program.
    assert(found == SPICETRUE);

    selected_body[i] = planet_id;
  }

  // Read Rotation setting
  std::vector<std::string> rotation_mode_list = ini_file.ReadVectorString(section, "rotation_mode", num_of_selected_body);

  CelestialInformation* celestial_info;
  celestial_info = new CelestialInformation(inertial_frame, aber_cor, center_obj, num_of_selected_body, selected_body, rotation_mode_list);

  // log setting
  celestial_info->is_log_enabled_ = ini_file.ReadEnable(section, INI_LOG_LABEL);

  return celestial_info;
}
