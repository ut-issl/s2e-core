/*
 * @file telescope.cpp
 * @brief Component emulation: Telescope
 */

#include "telescope.hpp"

#include <cassert>
#include <environment/global/physical_constants.hpp>
#include <math_physics/math/constants.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

using namespace std;
using namespace libra;

Telescope::Telescope(ClockGenerator* clock_generator, const libra::Quaternion& quaternion_b2c, const double sun_forbidden_angle_rad,
                     const double earth_forbidden_angle_rad, const double moon_forbidden_angle_rad, const int x_number_of_pix,
                     const int y_number_of_pix, const double pixel_size_m, const double focal_length_m, const double x_fov_per_pix_rad,
                     const double y_fov_per_pix_rad, const char* start_imaging_ymdhms, const double line_rate_sec, const int stage_mode,
                     const int number_of_lines_per_frame, const int number_of_frames_per_mission, size_t number_of_logged_stars,
                     const Attitude* attitude, const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information,
                     const Orbit* orbit, const SimulationTime* simulation_time)
    : Component(1, clock_generator),
      quaternion_b2c_(quaternion_b2c),
      sun_forbidden_angle_rad_(sun_forbidden_angle_rad),
      earth_forbidden_angle_rad_(earth_forbidden_angle_rad),
      moon_forbidden_angle_rad_(moon_forbidden_angle_rad),
      x_number_of_pix_(x_number_of_pix),
      y_number_of_pix_(y_number_of_pix),
      pixel_size_m_(pixel_size_m),
      focal_length_m_(focal_length_m),
      x_fov_per_pix_rad_(x_fov_per_pix_rad),
      y_fov_per_pix_rad_(y_fov_per_pix_rad),
      start_imaging_ymdhms_(start_imaging_ymdhms),
      line_rate_sec_(line_rate_sec),
      stage_mode_(stage_mode),
      number_of_lines_per_frame_(number_of_lines_per_frame),
      number_of_frames_per_mission_(number_of_frames_per_mission),
      number_of_logged_stars_(number_of_logged_stars),
      attitude_(attitude),
      hipparcos_(hipparcos),
      local_celestial_information_(local_celestial_information),
      orbit_(orbit),
      simulation_time_(simulation_time) {
  sscanf(start_imaging_ymdhms, "%d/%d/%d %d:%d:%lf", &start_imaging_year_, &start_imaging_month_, &start_imaging_day_, &start_imaging_hour_,
         &start_imaging_minute_, &start_imaging_sec_);
  jday(start_imaging_year_, start_imaging_month_, start_imaging_day_, start_imaging_hour_, start_imaging_minute_, start_imaging_sec_,
       start_imaging_jd_);

  is_sun_in_forbidden_angle = true;
  is_earth_in_forbidden_angle = true;
  is_moon_in_forbidden_angle = true;

  x_field_of_view_rad = x_number_of_pix_ * x_fov_per_pix_rad_;
  y_field_of_view_rad = y_number_of_pix_ * y_fov_per_pix_rad_;
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

  // Calculate imaging time
  double current_jd = simulation_time_->GetCurrentTime_jd();
  assert(start_imaging_jd_ > current_jd);
  double imaging_duration_sec = line_rate_sec_ * stage_mode_ * number_of_lines_per_frame_ * number_of_frames_per_mission_;
  double imaging_duration_day = imaging_duration_sec / 60.0 / 60.0 / 24.0;
  center_imaging_jd = start_imaging_jd_ + imaging_duration_day / 2.0;
  end_imaging_jd = start_imaging_jd_ + imaging_duration_day;
  assert(end_imaging_jd > current_jd + simulation_time_->GetEndTime_s() / 60.0 / 60.0 / 24.0);
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
  // No update when Hipparcos Catalogue was not read
  if (hipparcos_->IsCalcEnabled) ObserveStars();
  // Debug ******************************************************************
  //  sun_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("SUN"));
  //  earth_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("EARTH"));
  //  moon_pos_c = quaternion_b2c_.FrameConversion(dynamics_->celestial_->GetPositionFromSpacecraft_b_m("MOON"));
  // angle_sun = CalcAngleTwoVectors_rad(sight_direction_c_, sun_pos_c) * 180/libra::pi;
  // angle_earth = CalcAngleTwoVectors_rad(sight_direction_c_, earth_pos_c) * 180 / libra::pi; angle_moon =
  // CalcAngleTwoVectors_rad(sight_direction_c_, moon_pos_c) * 180 / libra::pi;
  //******************************************************************************
  // Direction calculation of ground point
  ObserveGroundPositionDeviation();
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

void Telescope::CalculateTargetGroundPosition() {
  // start imaging time, center, endにはじめてかかったときに位置とその時刻を記憶する
  // new calculation
  if (orbit_ != nullptr) {
    // center position in image sensor at component flame
    libra::Vector<3> target_center_c;
    target_center_c[0] = 1;
    target_center_c[1] = 0;
    target_center_c[2] = 0;
    // ymax position in image sensor at component flame
    libra::Vector<3> target_ymax_c;
    target_ymax_c[0] = 1;
    target_ymax_c[1] = ground_position_y_max_y_image_sensor_ * pixel_size_m_ / focal_length_m_;  // センサ内のy方向
    target_ymax_c[2] = ground_position_y_max_x_image_sensor_ * pixel_size_m_ / focal_length_m_;  // センサ内のx方向
    // ymin position in image sensor at component flame
    libra::Vector<3> target_ymin_c;
    target_ymin_c[0] = 1;
    target_ymin_c[1] = ground_position_y_min_y_image_sensor_ * pixel_size_m_ / focal_length_m_;  // センサ内のy方向
    target_ymin_c[2] = ground_position_y_min_x_image_sensor_ * pixel_size_m_ / focal_length_m_;  // センサ内のx方向
    // target_cをbにもどす
    libra::Vector<3> target_center_b = quaternion_b2c_.Conjugate().FrameConversion(target_center_c);
    libra::Vector<3> target_ymax_b = quaternion_b2c_.Conjugate().FrameConversion(target_ymax_c);
    libra::Vector<3> target_ymin_b = quaternion_b2c_.Conjugate().FrameConversion(target_ymin_c);
    // target_b->target_i
    Quaternion quaternion_b2i = attitude_->GetQuaternion_i2b().Conjugate();
    libra::Vector<3> target_center_i = quaternion_b2i.FrameConversion(target_center_b);
    libra::Vector<3> target_ymax_i = quaternion_b2i.FrameConversion(target_ymax_b);
    libra::Vector<3> target_ymin_i = quaternion_b2i.FrameConversion(target_ymin_b);
    // target_iをecefにもどす
    libra::Matrix<3, 3> dcm_i_to_ecef = local_celestial_information_->GetGlobalInformation().GetEarthRotation().GetDcmJ2000ToEcef();
    libra::Vector<3> direction_center_ecef_m = dcm_i_to_ecef * target_center_i;
    libra::Vector<3> direction_ymax_ecef_m = dcm_i_to_ecef * target_ymax_i;
    libra::Vector<3> direction_ymin_ecef_m = dcm_i_to_ecef * target_ymin_i;
    // spacecraft_ecef_m_を計算
    libra::Vector<3> current_spacecraft_position_ecef_m = orbit_->GetPosition_ecef_m();
    double k_center = (-(InnerProduct(current_spacecraft_position_ecef_m, direction_center_ecef_m)) -
                       sqrt(pow(InnerProduct(current_spacecraft_position_ecef_m, direction_center_ecef_m), 2) -
                            pow(direction_center_ecef_m.CalcNorm(), 2) *
                                (pow(current_spacecraft_position_ecef_m.CalcNorm(), 2) - pow(environment::earth_equatorial_radius_m, 2)))) /
                      pow(direction_center_ecef_m.CalcNorm(), 2);
    double k_ymax = (-(InnerProduct(current_spacecraft_position_ecef_m, direction_ymax_ecef_m)) -
                     sqrt(pow(InnerProduct(current_spacecraft_position_ecef_m, direction_ymax_ecef_m), 2) -
                          pow(direction_ymax_ecef_m.CalcNorm(), 2) *
                              (pow(current_spacecraft_position_ecef_m.CalcNorm(), 2) - pow(environment::earth_equatorial_radius_m, 2)))) /
                    pow(direction_ymax_ecef_m.CalcNorm(), 2);
    double k_ymin = (-(InnerProduct(current_spacecraft_position_ecef_m, direction_ymin_ecef_m)) -
                     sqrt(pow(InnerProduct(current_spacecraft_position_ecef_m, direction_ymin_ecef_m), 2) -
                          pow(direction_ymin_ecef_m.CalcNorm(), 2) *
                              (pow(current_spacecraft_position_ecef_m.CalcNorm(), 2) - pow(environment::earth_equatorial_radius_m, 2)))) /
                    pow(direction_ymin_ecef_m.CalcNorm(), 2);
    target_ground_position_center_ecef_m_ = current_spacecraft_position_ecef_m + k_center * direction_center_ecef_m;
    target_ground_position_ymax_ecef_m_ = current_spacecraft_position_ecef_m + k_ymax * direction_ymax_ecef_m;
    target_ground_position_ymin_ecef_m_ = current_spacecraft_position_ecef_m + k_ymin * direction_ymin_ecef_m;
  }
}

std::pair<double, double> Telescope::CalculateImagePosition(libra::Vector<3> target_ground_position_ecef_m_) {
  Quaternion quaternion_i2b = attitude_->GetQuaternion_i2b();
  libra::Vector<3> spacecraft_position_ecef_m = orbit_->GetPosition_ecef_m();
  libra::Matrix<3, 3> dcm_ecef_to_i = local_celestial_information_->GetGlobalInformation().GetEarthRotation().GetDcmJ2000ToEcef().Transpose();
  libra::Vector<3> direction_sat_ground_ecef_m = target_ground_position_ecef_m_ - spacecraft_position_ecef_m;
  libra::Vector<3> direction_sat_ground_i_m = dcm_ecef_to_i * direction_sat_ground_ecef_m;
  libra::Vector<3> direction_sat_ground_b_m = quaternion_i2b.FrameConversion(direction_sat_ground_i_m);
  libra::Vector<3> target_c = quaternion_b2c_.FrameConversion(direction_sat_ground_b_m);
  double angle_x = atan2(target_c[2], target_c[0]);
  double angle_y = atan2(target_c[1], target_c[0]);
  double ground_position_x_image_sensor_ = focal_length_m_ * tan(angle_x) / pixel_size_m_;
  double ground_position_y_image_sensor_ = focal_length_m_ * tan(angle_y) / pixel_size_m_;
  return std::make_pair(ground_position_x_image_sensor_, ground_position_y_image_sensor_);
}

void Telescope::ObserveGroundPositionDeviation() {
  // Orbit information is not available, so skip the ground position calculation
  if (orbit_ == nullptr) {
    return;
  }
  // Check if the ground point is in the image sensor
  if (ground_position_center_x_image_sensor_ > x_number_of_pix_ || ground_position_center_y_image_sensor_ > y_number_of_pix_) {
    ground_position_center_x_image_sensor_ = -1;
    ground_position_center_y_image_sensor_ = -1;
    return;
  }
  double current_jd = simulation_time_->GetCurrentTime_jd();
  double stage_time_sec = line_rate_sec_ * stage_mode_;
  if (start_imaging_jd_ <= current_jd <= start_imaging_jd_ + stage_time_sec) {
    if (startImagingFlag) {
      CalculateTargetGroundPosition();
      startImagingFlag = false;
    }
    std::pair<double, double> ground_position_center_image_sensor_ = CalculateImagePosition(target_ground_position_center_ecef_m_);
    ground_position_center_x_image_sensor_ = ground_position_center_image_sensor_.first;
    ground_position_center_y_image_sensor_ = ground_position_center_image_sensor_.second;
    std::pair<double, double> ground_position_y_max_image_sensor_ = CalculateImagePosition(target_ground_position_ymax_ecef_m_);
    ground_position_y_max_x_image_sensor_ = ground_position_y_max_image_sensor_.first;
    ground_position_y_max_y_image_sensor_ = ground_position_y_max_image_sensor_.second;
    std::pair<double, double> ground_position_y_min_image_sensor_ = CalculateImagePosition(target_ground_position_ymin_ecef_m_);
    ground_position_y_min_x_image_sensor_ = ground_position_y_min_image_sensor_.first;
    ground_position_y_min_y_image_sensor_ = ground_position_y_min_image_sensor_.second;
  }

  if (center_imaging_jd <= current_jd <= center_imaging_jd + stage_time_sec) {
    if (centerImagingFlag) {
      CalculateTargetGroundPosition();
      centerImagingFlag = false;
    }
    std::pair<double, double> ground_position_center_image_sensor_ = CalculateImagePosition(target_ground_position_center_ecef_m_);
    ground_position_center_x_image_sensor_ = ground_position_center_image_sensor_.first;
    ground_position_center_y_image_sensor_ = ground_position_center_image_sensor_.second;
    std::pair<double, double> ground_position_y_max_image_sensor_ = CalculateImagePosition(target_ground_position_ymax_ecef_m_);
    ground_position_y_max_x_image_sensor_ = ground_position_y_max_image_sensor_.first;
    ground_position_y_max_y_image_sensor_ = ground_position_y_max_image_sensor_.second;
    std::pair<double, double> ground_position_y_min_image_sensor_ = CalculateImagePosition(target_ground_position_ymin_ecef_m_);
    ground_position_y_min_x_image_sensor_ = ground_position_y_min_image_sensor_.first;
    ground_position_y_min_y_image_sensor_ = ground_position_y_min_image_sensor_.second;
  }

  if (end_imaging_jd <= current_jd <= end_imaging_jd + stage_time_sec) {
    if (endImagingFlag) {
      CalculateTargetGroundPosition();
      endImagingFlag = false;
    }
    std::pair<double, double> ground_position_center_image_sensor_ = CalculateImagePosition(target_ground_position_center_ecef_m_);
    ground_position_center_x_image_sensor_ = ground_position_center_image_sensor_.first;
    ground_position_center_y_image_sensor_ = ground_position_center_image_sensor_.second;
    std::pair<double, double> ground_position_y_max_image_sensor_ = CalculateImagePosition(target_ground_position_ymax_ecef_m_);
    ground_position_y_max_x_image_sensor_ = ground_position_y_max_image_sensor_.first;
    ground_position_y_max_y_image_sensor_ = ground_position_y_max_image_sensor_.second;
    std::pair<double, double> ground_position_y_min_image_sensor_ = CalculateImagePosition(target_ground_position_ymin_ecef_m_);
    ground_position_y_min_x_image_sensor_ = ground_position_y_min_image_sensor_.first;
    ground_position_y_min_y_image_sensor_ = ground_position_y_min_image_sensor_.second;
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
  str_tmp += WriteScalar(component_name + "ground_position_center_x", "pix");
  str_tmp += WriteScalar(component_name + "ground_position_center_y", "pix");
  str_tmp += WriteScalar(component_name + "ground_position_y_max_x", "pix");
  str_tmp += WriteScalar(component_name + "ground_position_y_max_y", "pix");
  str_tmp += WriteScalar(component_name + "ground_position_y_min_x", "pix");
  str_tmp += WriteScalar(component_name + "ground_position_y_min_y", "pix");
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
  str_tmp += WriteScalar(ground_position_center_x_image_sensor_);
  str_tmp += WriteScalar(ground_position_center_y_image_sensor_);
  str_tmp += WriteScalar(ground_position_y_max_x_image_sensor_);
  str_tmp += WriteScalar(ground_position_y_max_y_image_sensor_);
  str_tmp += WriteScalar(ground_position_y_min_x_image_sensor_);
  str_tmp += WriteScalar(ground_position_y_min_y_image_sensor_);
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

Telescope InitTelescope(ClockGenerator* clock_generator, int sensor_id, const string file_name, const Attitude* attitude,
                        const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information,
                        const SimulationTime* simulation_time, const Orbit* orbit) {
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

  libra::Quaternion quaternion_b2c;
  Telescope_conf.ReadQuaternion(TelescopeSection, "quaternion_b2c", quaternion_b2c);

  double sun_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * pi / 180;  // deg to rad
  double earth_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * pi / 180;  // deg to rad
  double moon_forbidden_angle_deg = Telescope_conf.ReadDouble(TelescopeSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * pi / 180;  // deg to rad

  int x_number_of_pix = Telescope_conf.ReadInt(TelescopeSection, "x_number_of_pixel");
  int y_number_of_pix = Telescope_conf.ReadInt(TelescopeSection, "y_number_of_pixel");

  double pixel_size_m = Telescope_conf.ReadDouble(TelescopeSection, "pixel_size_m");
  double focal_length_m = Telescope_conf.ReadDouble(TelescopeSection, "focal_length_m");

  double x_fov_per_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "x_fov_deg_per_pixel");
  double x_fov_per_pix_rad = x_fov_per_pix_deg * pi / 180;  // deg to rad
  double y_fov_per_pix_deg = Telescope_conf.ReadDouble(TelescopeSection, "y_fov_deg_per_pixel");
  double y_fov_per_pix_rad = y_fov_per_pix_deg * pi / 180;  // deg to rad

  std::string start_imaging_ymdhms = Telescope_conf.ReadString(TelescopeSection, "start_imaging_time_utc");
  double line_rate_sec = Telescope_conf.ReadDouble(TelescopeSection, "line_rate_sec");
  int stage_mode = Telescope_conf.ReadInt(TelescopeSection, "stage_mode");
  int number_of_lines_per_frame = Telescope_conf.ReadInt(TelescopeSection, "number_of_lines_per_frame");
  int number_of_frames_per_mission = Telescope_conf.ReadInt(TelescopeSection, "number_of_frames_per_mission");

  int number_of_logged_stars = Telescope_conf.ReadInt(TelescopeSection, "number_of_stars_for_log");

  Telescope telescope(clock_generator, quaternion_b2c, sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, x_number_of_pix,
                      y_number_of_pix, pixel_size_m, focal_length_m, x_fov_per_pix_rad, y_fov_per_pix_rad, start_imaging_ymdhms.c_str(),
                      line_rate_sec, stage_mode, number_of_lines_per_frame, number_of_frames_per_mission, number_of_logged_stars, attitude, hipparcos,
                      local_celestial_information, orbit, simulation_time);
  return telescope;
}
