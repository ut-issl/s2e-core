/**
 * @file celestial_information.cpp
 * @brief Class to manage the information related with the celestial bodies
 * @details This class uses SPICE to get the information of celestial bodies
 */

#include "celestial_information.hpp"

#include <SpiceUsr.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <sstream>

#include "library/logger/log_utility.hpp"

CelestialInformation::CelestialInformation(const std::string inertial_frame_name, const std::string aberration_correction_setting,
                                           const std::string center_body_name, const RotationMode rotation_mode,
                                           const unsigned int number_of_selected_body, int* selected_body_ids)
    : number_of_selected_bodies_(number_of_selected_body),
      selected_body_ids_(selected_body_ids),
      inertial_frame_name_(inertial_frame_name),
      center_body_name_(center_body_name),
      aberration_correction_setting_(aberration_correction_setting),
      rotation_mode_(rotation_mode) {
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
  earth_rotation_ = new CelestialRotation(rotation_mode_, center_body_name_);
}

CelestialInformation::CelestialInformation(const CelestialInformation& obj)
    : number_of_selected_bodies_(obj.number_of_selected_bodies_),
      inertial_frame_name_(obj.inertial_frame_name_),
      center_body_name_(obj.center_body_name_),
      aberration_correction_setting_(obj.aberration_correction_setting_),
      rotation_mode_(obj.rotation_mode_) {
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

void CelestialInformation::UpdateAllObjectsInfo(const double current_time_jd) {
  // Convert time
  SpiceDouble ephemeris_time;
  std::string julian_date = "jd " + std::to_string(current_time_jd);
  str2et_c(julian_date.c_str(), &ephemeris_time);

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
    GetPlanetOrbit(name_buffer, ephemeris_time, (SpiceDouble*)orbit_buffer_km);
    // Convert unit [km], [km/s] to [m], [m/s]
    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_center_i_m_[i * 3 + j] = orbit_buffer_km[j] * 1000.0;
      celestial_body_velocity_from_center_i_m_s_[i * 3 + j] = orbit_buffer_km[j + 3] * 1000.0;
    }
  }

  // Update celestial rotation
  earth_rotation_->Update(current_time_jd);
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
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
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
  const int kMaxNameLength = 100;
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
