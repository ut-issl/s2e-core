/**
 * @file gnss_receiver.cpp
 * @brief Class to emulate GNSS receiver
 */

#include "gnss_receiver.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/randomization/global_randomization.hpp>
#include <string>

GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, const int component_id, const std::string gnss_id,
                           const int max_channel, const AntennaModel antenna_model, const libra::Vector<3> antenna_position_b_m,
                           const libra::Quaternion quaternion_b2c, const double half_width_rad, const libra::Vector<3> noise_standard_deviation_m,
                           const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      max_channel_(max_channel),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      random_noise_i_x_(0.0, noise_standard_deviation_m[0], global_randomization.MakeSeed()),
      random_noise_i_y_(0.0, noise_standard_deviation_m[1], global_randomization.MakeSeed()),
      random_noise_i_z_(0.0, noise_standard_deviation_m[2], global_randomization.MakeSeed()),
      half_width_rad_(half_width_rad),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {}
GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                           const std::string gnss_id, const int max_channel, const AntennaModel antenna_model,
                           const libra::Vector<3> antenna_position_b_m, const libra::Quaternion quaternion_b2c, const double half_width_rad,
                           const libra::Vector<3> noise_standard_deviation_m, const Dynamics* dynamics, const GnssSatellites* gnss_satellites,
                           const SimulationTime* simulation_time)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      max_channel_(max_channel),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      random_noise_i_x_(0.0, noise_standard_deviation_m[0], global_randomization.MakeSeed()),
      random_noise_i_y_(0.0, noise_standard_deviation_m[1], global_randomization.MakeSeed()),
      random_noise_i_z_(0.0, noise_standard_deviation_m[2], global_randomization.MakeSeed()),
      half_width_rad_(half_width_rad),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {}

void GnssReceiver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  libra::Vector<3> pos_true_eci_ = dynamics_->GetOrbit().GetPosition_i_m();
  libra::Quaternion quaternion_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();

  CheckAntenna(pos_true_eci_, quaternion_i2b);

  if (is_gnss_visible_ == 1) {  // Antenna of GNSS-R can detect GNSS signal
    position_ecef_m_ = dynamics_->GetOrbit().GetPosition_ecef_m();
    position_llh_ = dynamics_->GetOrbit().GetLatLonAlt();
    velocity_ecef_m_s_ = dynamics_->GetOrbit().GetVelocity_ecef_m_s();
    AddNoise(pos_true_eci_, position_ecef_m_);

    utc_ = simulation_time_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simulation_time_->GetCurrentTime_jd());
  } else {
    // position information will not be updated in this case
    // only time information will be updated in this case (according to the receiver's internal clock)
    utc_ = simulation_time_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simulation_time_->GetCurrentTime_jd());
  }
}

void GnssReceiver::CheckAntenna(const libra::Vector<3> pos_true_eci_, libra::Quaternion quaternion_i2b) {
  if (antenna_model_ == AntennaModel::kSimple)
    CheckAntennaSimple(pos_true_eci_, quaternion_i2b);
  else if (antenna_model_ == AntennaModel::kCone)
    CheckAntennaCone(pos_true_eci_, quaternion_i2b);
}

void GnssReceiver::CheckAntennaSimple(const libra::Vector<3> pos_true_eci_, libra::Quaternion quaternion_i2b) {
  // Simplest model
  // GNSS sats are visible when antenna directs anti-earth direction
  // antenna normal vector at inertial frame
  libra::Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  libra::Vector<3> antenna_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_direction_c);
  libra::Vector<3> antenna_direction_i = quaternion_i2b.InverseFrameConversion(antenna_direction_b);

  double inner = InnerProduct(pos_true_eci_, antenna_direction_i);
  if (inner <= 0)
    is_gnss_visible_ = 0;
  else
    is_gnss_visible_ = 1;
}

void GnssReceiver::CheckAntennaCone(const libra::Vector<3> pos_true_eci_, libra::Quaternion quaternion_i2b) {
  // Cone model
  libra::Vector<3> gnss_sat_pos_i, ant_pos_i, antenna_to_satellite_i_m, ant2gnss_i_n, sat2ant_i;
  gnss_information_list_.clear();

  // antenna normal vector at inertial frame
  libra::Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  libra::Vector<3> antenna_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_direction_c);
  libra::Vector<3> antenna_direction_i = quaternion_i2b.InverseFrameConversion(antenna_direction_b);

  sat2ant_i = quaternion_i2b.InverseFrameConversion(antenna_position_b_m_);
  ant_pos_i = pos_true_eci_ + sat2ant_i;

  // initialize
  visible_satellite_number_ = 0;

  int gnss_num = gnss_satellites_->GetNumOfSatellites();

  for (int i = 0; i < gnss_num; i++) {
    // check if gnss ID is compatible with the receiver
    std::string id_tmp = gnss_satellites_->GetIDFromIndex(i);
    if (gnss_id_.find(id_tmp[0]) == std::string::npos) continue;

    // compute direction from sat to gnss in body-fixed frame
    gnss_sat_pos_i = gnss_satellites_->GetSatellitePositionEci(i);
    antenna_to_satellite_i_m = gnss_sat_pos_i - ant_pos_i;
    double normalizer = 1 / antenna_to_satellite_i_m.CalcNorm();
    ant2gnss_i_n = normalizer * antenna_to_satellite_i_m;

    // check gnss sats are visible from antenna
    double Re = environment::earth_equatorial_radius_m;
    double inner1 = InnerProduct(ant_pos_i, gnss_sat_pos_i);
    int is_visible_ant2gnss = 0;
    if (inner1 > 0)
      is_visible_ant2gnss = 1;
    else {
      Vector<3> tmp = ant_pos_i + InnerProduct(-ant_pos_i, ant2gnss_i_n) * antenna_to_satellite_i_m;
      if (tmp.CalcNorm() < Re)
        // There is earth between antenna and gnss
        is_visible_ant2gnss = 0;
      else
        // There is not earth between antenna and gnss
        is_visible_ant2gnss = 1;
    }

    double inner2 = InnerProduct(antenna_direction_i, ant2gnss_i_n);
    if (inner2 > cos(half_width_rad_ * libra::deg_to_rad) && is_visible_ant2gnss) {
      // is visible
      visible_satellite_number_++;
      SetGnssInfo(antenna_to_satellite_i_m, quaternion_i2b, id_tmp);
    }
  }

  if (visible_satellite_number_ > 0)
    is_gnss_visible_ = 1;
  else
    is_gnss_visible_ = 0;
}

void GnssReceiver::SetGnssInfo(libra::Vector<3> antenna_to_satellite_i_m, libra::Quaternion quaternion_i2b, std::string gnss_id) {
  libra::Vector<3> ant2gnss_b, ant2gnss_c;

  ant2gnss_b = quaternion_i2b.FrameConversion(antenna_to_satellite_i_m);
  ant2gnss_c = quaternion_b2c_.FrameConversion(ant2gnss_b);

  double dist = ant2gnss_c.CalcNorm();
  double lon = AcTan(ant2gnss_c[1], ant2gnss_c[0]);
  double lat = AcTan(ant2gnss_c[2], sqrt(pow(ant2gnss_c[0], 2.0) + pow(ant2gnss_c[1], 2.0)));

  GnssInfo gnss_info_new = {gnss_id, lat, lon, dist};
  gnss_information_list_.push_back(gnss_info_new);
}

void GnssReceiver::AddNoise(libra::Vector<3> position_true_i_m, libra::Vector<3> position_true_ecef_m) {
  // Simplest noise model
  position_eci_m_[0] = position_true_i_m[0] + random_noise_i_x_;
  position_eci_m_[1] = position_true_i_m[1] + random_noise_i_y_;
  position_eci_m_[2] = position_true_i_m[2] + random_noise_i_z_;

  // FIXME: noise in ECI frame is added to ECEF frame value
  position_ecef_m_[0] = position_true_ecef_m[0] + random_noise_i_x_;
  position_ecef_m_[1] = position_true_ecef_m[1] + random_noise_i_y_;
  position_ecef_m_[2] = position_true_ecef_m[2] + random_noise_i_z_;
}

void GnssReceiver::ConvertJulianDayToGPSTime(const double julian_day) {
  const double kJulianDayAtGPSTimeZero = 2444244.5;  // corresponds to 1980/1/5 midnight
  const double kDayInWeek = 7.0;
  // const double kSecInWeek = 604800.0;
  const double kSecInDay = 86400.0;

  // compute ToW from current julian_day
  // note:"gps_time_week_ " computed in this method is larger than 1024
  double elapsed_day = julian_day - kJulianDayAtGPSTimeZero;
  gps_time_week_ = (unsigned int)(elapsed_day / kDayInWeek);
  gps_time_s_ = (elapsed_day - (double)(gps_time_week_)*kDayInWeek) * kSecInDay;
}

std::string GnssReceiver::GetLogHeader() const  // For logs
{
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(component_id_));
  std::string sensor_name = "gnss_receiver" + sensor_id + "_";

  str_tmp += WriteScalar(sensor_name + "measured_utc_time_year");
  str_tmp += WriteScalar(sensor_name + "measured_utc_time_month");
  str_tmp += WriteScalar(sensor_name + "measured_utc_time_day");
  str_tmp += WriteScalar(sensor_name + "measured_utc_time_hour");
  str_tmp += WriteScalar(sensor_name + "measured_utc_time_min");
  str_tmp += WriteScalar(sensor_name + "measured_utc_time_sec");
  str_tmp += WriteVector(sensor_name + "measured_position", "eci", "m", 3);
  str_tmp += WriteVector(sensor_name + "measured_velocity", "ecef", "m/s", 3);
  str_tmp += WriteScalar(sensor_name + "measured_latitude", "rad");
  str_tmp += WriteScalar(sensor_name + "measured_longitude", "rad");
  str_tmp += WriteScalar(sensor_name + "measured_altitude", "m");
  str_tmp += WriteScalar(sensor_name + "satellite_visible_flag");
  str_tmp += WriteScalar(sensor_name + "number_of_visible_satellites");

  return str_tmp;
}

std::string GnssReceiver::GetLogValue() const  // For logs
{
  std::string str_tmp = "";
  str_tmp += WriteScalar(utc_.year);
  str_tmp += WriteScalar(utc_.month);
  str_tmp += WriteScalar(utc_.day);
  str_tmp += WriteScalar(utc_.hour);
  str_tmp += WriteScalar(utc_.minute);
  str_tmp += WriteScalar(utc_.second);
  str_tmp += WriteVector(position_eci_m_, 10);
  str_tmp += WriteVector(velocity_ecef_m_s_, 10);
  str_tmp += WriteScalar(position_llh_[0], 10);
  str_tmp += WriteScalar(position_llh_[1], 10);
  str_tmp += WriteScalar(position_llh_[2], 10);
  str_tmp += WriteScalar(is_gnss_visible_);
  str_tmp += WriteScalar(visible_satellite_number_);

  return str_tmp;
}
