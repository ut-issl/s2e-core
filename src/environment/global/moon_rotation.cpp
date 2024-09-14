/**
 * @file moon_rotation.cpp
 * @brief Class to calculate the moon rotation
 */

#include "moon_rotation.hpp"

#include <SpiceUsr.h>

#include <math_physics/math/constants.hpp>
#include <math_physics/planet_rotation/moon_rotation_utilities.hpp>

MoonRotation::MoonRotation(const CelestialInformation& celestial_information, MoonRotationMode mode)
    : mode_(mode), celestial_information_(celestial_information) {
  dcm_j2000_to_mcmf_ = s2e::math::MakeIdentityMatrix<3>();
}

void MoonRotation::Update(const SimulationTime& simulation_time) {
  if (mode_ == MoonRotationMode::kSimple) {
    s2e::math::Vector<3> moon_position_eci_m = celestial_information_.GetPositionFromSelectedBody_i_m("MOON", "EARTH");
    s2e::math::Vector<3> moon_velocity_eci_m_s = celestial_information_.GetVelocityFromSelectedBody_i_m_s("MOON", "EARTH");
    dcm_j2000_to_mcmf_ = s2e::planet_rotation::CalcDcmEciToPrincipalAxis(moon_position_eci_m, moon_velocity_eci_m_s);
  } else if (mode_ == MoonRotationMode::kIauMoon) {
    ConstSpiceChar from[] = "J2000";
    ConstSpiceChar to[] = "IAU_MOON";
    SpiceDouble et = simulation_time.GetCurrentEphemerisTime();
    SpiceDouble state_transition_matrix[6][6];
    sxform_c(from, to, et, state_transition_matrix);
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        dcm_j2000_to_mcmf_[i][j] = state_transition_matrix[i][j];
      }
    }
  } else {
    dcm_j2000_to_mcmf_ = s2e::math::MakeIdentityMatrix<3>();
  }
}

MoonRotationMode ConvertMoonRotationMode(const std::string mode) {
  MoonRotationMode rotation_mode;
  if (mode == "IDLE") {
    rotation_mode = MoonRotationMode::kIdle;
  } else if (mode == "SIMPLE") {
    rotation_mode = MoonRotationMode::kSimple;
  } else if (mode == "IAU_MOON") {
    rotation_mode = MoonRotationMode::kIauMoon;
  } else  // if rotation_mode is neither Idle, Simple, nor Full, set rotation_mode to Idle
  {
    rotation_mode = MoonRotationMode::kIdle;
  }

  return rotation_mode;
}