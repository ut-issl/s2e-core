/*
 * @file telescope.cpp
 * @brief Component emulation: Telescope
 */

#include "telescope.hpp"

#include <cassert>
#include <library/math/constants.hpp>

using namespace std;
using namespace libra;

Telescope::Telescope(ClockGenerator* clock_generator, const libra::Quaternion& quaternion_b2c, const double sun_forbidden_angle_rad,
                     const double earth_forbidden_angle_rad, const double moon_forbidden_angle_rad, const int x_number_of_pix,
                     const int y_number_of_pix, const double x_fov_per_pix, const double y_fov_per_pix, size_t number_of_logged_stars,
                     const Attitude* attitude, const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information)
    : Component(1, clock_generator),
      quaternion_b2c_(quaternion_b2c),
      sun_forbidden_angle_rad_(sun_forbidden_angle_rad),
      earth_forbidden_angle_rad_(earth_forbidden_angle_rad),
      moon_forbidden_angle_rad_(moon_forbidden_angle_rad),
      x_number_of_pix_(x_number_of_pix),
      y_number_of_pix_(y_number_of_pix),
      x_fov_per_pix_(x_fov_per_pix),
      y_fov_per_pix_(y_fov_per_pix),
      number_of_logged_stars_(number_of_logged_stars),
      attitude_(attitude),
      hipparcos_(hipparcos),
      local_celestial_information_(local_celestial_information) {
  is_sun_in_forbidden_angle = true;
  is_earth_in_forbidden_angle = true;
  is_moon_in_forbidden_angle = true;

  x_field_of_view_rad = x_number_of_pix_ * x_fov_per_pix_;
  y_field_of_view_rad = y_number_of_pix_ * y_fov_per_pix_;
  assert(x_field_of_view_rad < libra::pi_2);  // Avoid the case that the field of view is over 90 degrees
  assert(y_field_of_view_rad < libra::pi_2);

  sight_direction_c_ = Vector<3>(0);
  sight_direction_c_[0] = 1;  // (1,0,0) at component frame, Sight direction vector

  // Set 0 when t=0
  for (size_t i = 0; i < number_of_logged_stars_; i++) {
    Star star;
    star.hipparcos_data.hipparcos_id = -1;
    star.hipparcos_data.visible_magnitude = -1;
    star.hipparcos_data.right_ascension_deg = -1;
    star.hipparcos_data.declination_deg = -1;
    star.position_image_sensor[0] = -1;
    star.position_image_sensor[1] = -1;

    star_list_in_sight.push_back(star);
  }
}

Telescope::~Telescope() {}

void Telescope::MainRoutine(const int time_count) {
  UNUSED(time_count);
  // Check forbidden angle
  is_sun_in_forbidden_angle = JudgeForbiddenAngle(local_celestial_information_->GetPositionFromSpacecraft_b_m("SUN"), sun_forbidden_angle_rad_);
  is_earth_in_forbidden_angle = JudgeForbiddenAngle(local_celestial_information_->GetPositionFromSpacecraft_b_m("EARTH"), earth_forbidden_angle_rad_);
  is_moon_in_forbidden_angle = JudgeForbiddenAngle(local_celestial_information_->GetPositionFromSpacecraft_b_m("MOON"), moon_forbidden_angle_rad_);
  // Position calculation of celestial bodies from CelesInfo
  Observe(sun_position_image_sensor, local_celestial_information_->GetPositionFromSpacecraft_b_m("SUN"));
  Observe(earth_position_image_sensor, local_celestial_information_->GetPositionFromSpacecraft_b_m("EARTH"));
  Observe(moon_position_image_sensor, local_celestial_information_->GetPositionFromSpacecraft_b_m("MOON"));
  // Position calculation of stars from Hipparcos Catalogue
  // No update when Hipparocos Catalogue was not readed
  if (hipparcos_->IsCalcEnabled) ObserveStars();
  // Debug ******************************************************************
  //  sun_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("SUN"));
  //  earth_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("EARTH"));
  //  moon_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("MOON"));
  // angle_sun = CalcAngleTwoVectors_rad(sight_direction_c_, sun_pos_c) * 180/libra::pi;
  // angle_earth = CalcAngleTwoVectors_rad(sight_direction_c_, earth_pos_c) * 180 / libra::pi; angle_moon =
  // CalcAngleTwoVectors_rad(sight_direction_c_, moon_pos_c) * 180 / libra::pi;
  //******************************************************************************
}

bool Telescope::JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle) {
  libra::Quaternion q_c2b = quaternion_b2c_.Conjugate();
  libra::Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double angle_rad = libra::CalcAngleTwoVectors_rad(target_b, sight_b);
  if (angle_rad < forbidden_angle) {
    return true;
  } else
    return false;
}

void Telescope::Observe(libra::Vector<2>& position_image_sensor, const libra::Vector<3, double> target_b) {
  libra::Vector<3, double> target_c = quaternion_b2c_.FrameConversion(target_b);
  double arg_x = atan2(target_c[2], target_c[0]);  // Angle from X-axis on XZ plane in the component frame
  double arg_y = atan2(target_c[1], target_c[0]);  // Angle from X-axis on XY plane in the component frame

  if (abs(arg_x) < x_field_of_view_rad && abs(arg_y) < y_field_of_view_rad) {
    position_image_sensor[0] = x_number_of_pix_ / 2 * tan(arg_x) / tan(x_field_of_view_rad) + x_number_of_pix_ / 2;
    position_image_sensor[1] = y_number_of_pix_ / 2 * tan(arg_y) / tan(y_field_of_view_rad) + y_number_of_pix_ / 2;
  } else {  // Return -1 when the body is in the out of FoV
    position_image_sensor[0] = -1;
    position_image_sensor[1] = -1;
  }
}

void Telescope::ObserveStars() {
  Quaternion quaternion_i2b = attitude_->GetQuaternion_i2b();

  star_list_in_sight.clear();  // Clear first
  size_t count = 0;            // Counter for while loop

  while (star_list_in_sight.size() < number_of_logged_stars_) {
    libra::Vector<3> target_b = hipparcos_->GetStarDirection_b(count, quaternion_i2b);
    libra::Vector<3> target_c = quaternion_b2c_.FrameConversion(target_b);

    double arg_x = atan2(target_c[2], target_c[0]);  // Angle from X-axis on XZ plane in the component frame
    double arg_y = atan2(target_c[1], target_c[0]);  // Angle from X-axis on XY plane in the component frame

    if (abs(arg_x) <= x_field_of_view_rad && abs(arg_y) <= y_field_of_view_rad) {
      Star star;
      star.hipparcos_data.hipparcos_id = hipparcos_->GetHipparcosId(count);
      star.hipparcos_data.visible_magnitude = hipparcos_->GetVisibleMagnitude(count);
      star.hipparcos_data.right_ascension_deg = hipparcos_->GetRightAscension_deg(count);
      star.hipparcos_data.declination_deg = hipparcos_->GetDeclination_deg(count);
      star.position_image_sensor[0] = x_number_of_pix_ / 2.0 * tan(arg_x) / tan(x_field_of_view_rad) + x_number_of_pix_ / 2.0;
      star.position_image_sensor[1] = y_number_of_pix_ / 2.0 * tan(arg_y) / tan(y_field_of_view_rad) + y_number_of_pix_ / 2.0;

      star_list_in_sight.push_back(star);
    }

    count++;

    // If read all catalogue, fill -1 and break the loop
    if (count >= hipparcos_->GetCatalogueSize()) {
      while (star_list_in_sight.size() < number_of_logged_stars_) {
        Star star;
        star.hipparcos_data.hipparcos_id = -1;
        star.hipparcos_data.visible_magnitude = -1;
        star.hipparcos_data.right_ascension_deg = -1;
        star.hipparcos_data.declination_deg = -1;
        star.position_image_sensor[0] = -1;
        star.position_image_sensor[1] = -1;

        star_list_in_sight.push_back(star);
      }

      break;
    }
  }
}

string Telescope::GetLogHeader() const {
  string str_tmp = "";

  std::string component_name = "telescope_";

  str_tmp += WriteScalar(component_name + "sun_in_exclusion_angle", "");
  str_tmp += WriteScalar(component_name + "earth_in_exclusion_angle", "");
  str_tmp += WriteScalar(component_name + "moon_in_exclusion_angle", "");
  str_tmp += WriteVector(component_name + "sun_position", "img", "pix", 2);
  str_tmp += WriteVector(component_name + "earth_position", "img", "pix", 2);
  str_tmp += WriteVector(component_name + "moon_position", "img", "pix", 2);
  // When Hipparcos Catalogue was not read, no output of ObserveStars
  if (hipparcos_->IsCalcEnabled) {
    for (size_t i = 0; i < number_of_logged_stars_; i++) {
      str_tmp += WriteScalar(component_name + "hipparcos_id (" + to_string(i) + ")", " ");
      str_tmp += WriteScalar(component_name + "visible_magnitude (" + to_string(i) + ")", " ");
      str_tmp += WriteVector(component_name + "star_position (" + to_string(i) + ")", "img", "pix", 2);
    }
  }

  // Debug output **********************************************
  //  str_tmp += WriteScalar("angle_sun", "");
  //  str_tmp += WriteScalar("angle_earth", "");
  //  str_tmp += WriteScalar("angle_moon", "");
  //**********************************************************
  return str_tmp;
}

string Telescope::GetLogValue() const {
  string str_tmp = "";
  str_tmp += WriteScalar(is_sun_in_forbidden_angle);
  str_tmp += WriteScalar(is_earth_in_forbidden_angle);
  str_tmp += WriteScalar(is_moon_in_forbidden_angle);
  str_tmp += WriteVector(sun_position_image_sensor);
  str_tmp += WriteVector(earth_position_image_sensor);
  str_tmp += WriteVector(moon_position_image_sensor);
  // When Hipparcos Catalogue was not read, no output of ObserveStars
  if (hipparcos_->IsCalcEnabled) {
    for (size_t i = 0; i < number_of_logged_stars_; i++) {
      str_tmp += WriteScalar(star_list_in_sight[i].hipparcos_data.hipparcos_id);
      str_tmp += WriteScalar(star_list_in_sight[i].hipparcos_data.visible_magnitude);
      str_tmp += WriteVector(star_list_in_sight[i].position_image_sensor);
    }
  }

  // Debug output **********************************************
  //  str_tmp += WriteScalar(angle_sun);
  //  str_tmp += WriteScalar(angle_earth);
  //  str_tmp += WriteScalar(angle_moon);
  //**********************************************************
  return str_tmp;
}
