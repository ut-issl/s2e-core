/*
 * @file telescope.cpp
 * @brief Component emulation: Telescope
 */

#include "telescope.hpp"

#include <cassert>
#include <library/math/constants.hpp>

using namespace std;
using namespace libra;

Telescope::Telescope(ClockGenerator* clock_gen, libra::Quaternion& q_b2c, double sun_forbidden_angle, double earth_forbidden_angle,
                     double moon_forbidden_angle, int x_num_of_pix, int y_num_of_pix, double x_fov_par_pix, double y_fov_par_pix,
                     size_t num_of_logged_stars, const Attitude* attitude, const HipparcosCatalogue* hipp,
                     const LocalCelestialInformation* local_celes_info)
    : Component(1, clock_gen),
      q_b2c_(q_b2c),
      sun_forbidden_angle_(sun_forbidden_angle),
      earth_forbidden_angle_(earth_forbidden_angle),
      moon_forbidden_angle_(moon_forbidden_angle),
      x_num_of_pix_(x_num_of_pix),
      y_num_of_pix_(y_num_of_pix),
      x_fov_par_pix_(x_fov_par_pix),
      y_fov_par_pix_(y_fov_par_pix),
      num_of_logged_stars_(num_of_logged_stars),
      attitude_(attitude),
      hipp_(hipp),
      local_celes_info_(local_celes_info) {
  is_sun_in_forbidden_angle = true;
  is_earth_in_forbidden_angle = true;
  is_moon_in_forbidden_angle = true;

  x_field_of_view_rad = x_num_of_pix_ * x_fov_par_pix_;
  y_field_of_view_rad = y_num_of_pix_ * y_fov_par_pix_;
  assert(x_field_of_view_rad < libra::pi_2);  // Avoid the case that the field of view is over 90 degrees
  assert(y_field_of_view_rad < libra::pi_2);

  sight_ = Vector<3>(0);
  sight_[0] = 1;  // (1,0,0) at component frame, Sight direction vector

  // Set 0 when t=0
  for (size_t i = 0; i < num_of_logged_stars_; i++) {
    Star star;
    star.hipdata.hipparcos_id = -1;
    star.hipdata.visible_magnitude = -1;
    star.hipdata.right_ascension_deg = -1;
    star.hipdata.declination_deg = -1;
    star.pos_imgsensor[0] = -1;
    star.pos_imgsensor[1] = -1;

    star_in_sight.push_back(star);
  }
}

Telescope::~Telescope() {}

void Telescope::MainRoutine(int count) {
  UNUSED(count);
  // Check forbidden angle
  is_sun_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPositionFromSpacecraft_b_m("SUN"), sun_forbidden_angle_);
  is_earth_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPositionFromSpacecraft_b_m("EARTH"), earth_forbidden_angle_);
  is_moon_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPositionFromSpacecraft_b_m("MOON"), moon_forbidden_angle_);
  // Position calculation of celestial bodies from CelesInfo
  Observe(sun_pos_imgsensor, local_celes_info_->GetPositionFromSpacecraft_b_m("SUN"));
  Observe(earth_pos_imgsensor, local_celes_info_->GetPositionFromSpacecraft_b_m("EARTH"));
  Observe(moon_pos_imgsensor, local_celes_info_->GetPositionFromSpacecraft_b_m("MOON"));
  // Position calculation of stars from Hipparcos Catalogue
  // No update when Hipparocos Catalogue was not readed
  if (hipp_->IsCalcEnabled) ObserveStars();
  // Debug ******************************************************************
  //  sun_pos_c = q_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("SUN"));
  //  earth_pos_c = q_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("EARTH"));
  //  moon_pos_c = q_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("MOON"));
  // angle_sun = CalcAngleTwoVectors_rad(sight_, sun_pos_c) * 180/libra::pi;
  // angle_earth = CalcAngleTwoVectors_rad(sight_, earth_pos_c) * 180 / libra::pi; angle_moon = CalcAngleTwoVectors_rad(sight_, moon_pos_c) * 180 /
  // libra::pi;
  //******************************************************************************
}

bool Telescope::JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle) {
  Quaternion q_c2b = q_b2c_.Conjugate();
  Vector<3> sight_b = q_c2b.FrameConversion(sight_);
  double angle_rad = libra::CalcAngleTwoVectors_rad(target_b, sight_b);
  if (angle_rad < forbidden_angle) {
    return true;
  } else
    return false;
}

void Telescope::Observe(Vector<2>& pos_imgsensor, const Vector<3, double> target_b) {
  Vector<3, double> target_c = q_b2c_.FrameConversion(target_b);
  double arg_x = atan2(target_c[2], target_c[0]);  // Angle from X-axis on XZ plane in the component frame
  double arg_y = atan2(target_c[1], target_c[0]);  // Angle from X-axis on XY plane in the component frame

  if (abs(arg_x) < x_field_of_view_rad && abs(arg_y) < y_field_of_view_rad) {
    pos_imgsensor[0] = x_num_of_pix_ / 2 * tan(arg_x) / tan(x_field_of_view_rad) + x_num_of_pix_ / 2;
    pos_imgsensor[1] = y_num_of_pix_ / 2 * tan(arg_y) / tan(y_field_of_view_rad) + y_num_of_pix_ / 2;
  } else {  // Return -1 when the body is in the out of FoV
    pos_imgsensor[0] = -1;
    pos_imgsensor[1] = -1;
  }
}

void Telescope::ObserveStars() {
  Quaternion q_i2b = attitude_->GetQuaternion_i2b();

  star_in_sight.clear();  // Clear first
  int count = 0;          // Counter for while loop

  while (star_in_sight.size() < num_of_logged_stars_) {
    Vector<3> target_b = hipp_->GetStarDirection_b(count, q_i2b);
    Vector<3> target_c = q_b2c_.FrameConversion(target_b);

    double arg_x = atan2(target_c[2], target_c[0]);  // Angle from X-axis on XZ plane in the component frame
    double arg_y = atan2(target_c[1], target_c[0]);  // Angle from X-axis on XY plane in the component frame

    if (abs(arg_x) <= x_field_of_view_rad && abs(arg_y) <= y_field_of_view_rad) {
      Star star;
      star.hipdata.hipparcos_id = hipp_->GetHipparcosId(count);
      star.hipdata.visible_magnitude = hipp_->GetVisibleMagnitude(count);
      star.hipdata.right_ascension_deg = hipp_->GetRightAscension_deg(count);
      star.hipdata.declination_deg = hipp_->GetDeclination_deg(count);
      star.pos_imgsensor[0] = x_num_of_pix_ / 2.0 * tan(arg_x) / tan(x_field_of_view_rad) + x_num_of_pix_ / 2.0;
      star.pos_imgsensor[1] = y_num_of_pix_ / 2.0 * tan(arg_y) / tan(y_field_of_view_rad) + y_num_of_pix_ / 2.0;

      star_in_sight.push_back(star);
    }

    count++;

    // If read all catalogue, fill -1 and break the loop
    if (count >= hipp_->GetCatalogueSize()) {
      while (star_in_sight.size() < num_of_logged_stars_) {
        Star star;
        star.hipdata.hipparcos_id = -1;
        star.hipdata.visible_magnitude = -1;
        star.hipdata.right_ascension_deg = -1;
        star.hipdata.declination_deg = -1;
        star.pos_imgsensor[0] = -1;
        star.pos_imgsensor[1] = -1;

        star_in_sight.push_back(star);
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
  if (hipp_->IsCalcEnabled) {
    for (size_t i = 0; i < num_of_logged_stars_; i++) {
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
  str_tmp += WriteVector(sun_pos_imgsensor);
  str_tmp += WriteVector(earth_pos_imgsensor);
  str_tmp += WriteVector(moon_pos_imgsensor);
  // When Hipparcos Catalogue was not read, no output of ObserveStars
  if (hipp_->IsCalcEnabled) {
    for (size_t i = 0; i < num_of_logged_stars_; i++) {
      str_tmp += WriteScalar(star_in_sight[i].hipdata.hipparcos_id);
      str_tmp += WriteScalar(star_in_sight[i].hipdata.visible_magnitude);
      str_tmp += WriteVector(star_in_sight[i].pos_imgsensor);
    }
  }

  // Debug output **********************************************
  //  str_tmp += WriteScalar(angle_sun);
  //  str_tmp += WriteScalar(angle_earth);
  //  str_tmp += WriteScalar(angle_moon);
  //**********************************************************
  return str_tmp;
}
