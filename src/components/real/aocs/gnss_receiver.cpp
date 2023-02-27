/**
 * @file gnss_receiver.cpp
 * @brief Class to emulate GNSS receiver
 */

#include "gnss_receiver.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/randomization/global_randomization.hpp>
#include <string>

GNSSReceiver::GNSSReceiver(const int prescaler, ClockGenerator* clock_generator, const int id, const std::string gnss_id, const int ch_max,
                           const AntennaModel antenna_model, const Vector<3> ant_pos_b, const Quaternion q_b2c, const double half_width,
                           const Vector<3> noise_std, const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simtime)
    : Component(prescaler, clock_generator),
      id_(id),
      ch_max_(ch_max),
      antenna_position_b_(ant_pos_b),
      q_b2c_(q_b2c),
      nrs_eci_x_(0.0, noise_std[0], global_randomization.MakeSeed()),
      nrs_eci_y_(0.0, noise_std[1], global_randomization.MakeSeed()),
      nrs_eci_z_(0.0, noise_std[2], global_randomization.MakeSeed()),
      half_width_(half_width),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simtime_(simtime) {}
GNSSReceiver::GNSSReceiver(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int id, const std::string gnss_id,
                           const int ch_max, const AntennaModel antenna_model, const Vector<3> ant_pos_b, const Quaternion q_b2c,
                           const double half_width, const Vector<3> noise_std, const Dynamics* dynamics, const GnssSatellites* gnss_satellites,
                           const SimulationTime* simtime)
    : Component(prescaler, clock_generator, power_port),
      id_(id),
      ch_max_(ch_max),
      antenna_position_b_(ant_pos_b),
      q_b2c_(q_b2c),
      nrs_eci_x_(0.0, noise_std[0], global_randomization.MakeSeed()),
      nrs_eci_y_(0.0, noise_std[1], global_randomization.MakeSeed()),
      nrs_eci_z_(0.0, noise_std[2], global_randomization.MakeSeed()),
      half_width_(half_width),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simtime_(simtime) {}

void GNSSReceiver::MainRoutine(int count) {
  UNUSED(count);

  Vector<3> pos_true_eci_ = dynamics_->GetOrbit().GetPosition_i_m();
  Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();

  CheckAntenna(pos_true_eci_, q_i2b);

  if (is_gnss_sats_visible_ == 1) {  // Antenna of GNSS-R can detect GNSS signal
    position_ecef_ = dynamics_->GetOrbit().GetPosition_ecef_m();
    position_llh_ = dynamics_->GetOrbit().GetLatLonAlt();
    velocity_ecef_ = dynamics_->GetOrbit().GetVelocity_ecef_m_s();
    AddNoise(pos_true_eci_, position_ecef_);

    utc_ = simtime_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simtime_->GetCurrentTime_jd());
  } else {
    // position information will not be updated in this case
    // only time information will be updated in this case (according to the receiver's internal clock)
    utc_ = simtime_->GetCurrentUtc();
    ConvertJulianDayToGPSTime(simtime_->GetCurrentTime_jd());
  }
}

void GNSSReceiver::CheckAntenna(const Vector<3> pos_true_eci_, Quaternion q_i2b) {
  if (antenna_model_ == SIMPLE)
    CheckAntennaSimple(pos_true_eci_, q_i2b);
  else if (antenna_model_ == CONE)
    CheckAntennaCone(pos_true_eci_, q_i2b);
}

void GNSSReceiver::CheckAntennaSimple(const Vector<3> pos_true_eci_, Quaternion q_i2b) {
  // Simplest model
  // GNSS sats are visible when antenna directs anti-earth direction
  // antenna normal vector at inertial frame
  Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  Vector<3> antenna_direction_b = q_b2c_.InverseFrameConversion(antenna_direction_c);
  Vector<3> antenna_direction_i = q_i2b.InverseFrameConversion(antenna_direction_b);

  double inner = InnerProduct(pos_true_eci_, antenna_direction_i);
  if (inner <= 0)
    is_gnss_sats_visible_ = 0;
  else
    is_gnss_sats_visible_ = 1;
}

void GNSSReceiver::CheckAntennaCone(const Vector<3> pos_true_eci_, Quaternion q_i2b) {
  // Cone model
  Vector<3> gnss_sat_pos_i, ant_pos_i, ant2gnss_i, ant2gnss_i_n, sat2ant_i;
  vec_gnssinfo_.clear();

  // antenna normal vector at inertial frame
  Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  Vector<3> antenna_direction_b = q_b2c_.InverseFrameConversion(antenna_direction_c);
  Vector<3> antenna_direction_i = q_i2b.InverseFrameConversion(antenna_direction_b);

  sat2ant_i = q_i2b.InverseFrameConversion(antenna_position_b_);
  ant_pos_i = pos_true_eci_ + sat2ant_i;

  // initialize
  gnss_sats_visible_num_ = 0;

  int gnss_num = gnss_satellites_->GetNumOfSatellites();

  for (int i = 0; i < gnss_num; i++) {
    // check if gnss ID is compatible with the receiver
    std::string id_tmp = gnss_satellites_->GetIDFromIndex(i);
    if (gnss_id_.find(id_tmp[0]) == std::string::npos) continue;

    // compute direction from sat to gnss in body-fixed frame
    gnss_sat_pos_i = gnss_satellites_->GetSatellitePositionEci(i);
    ant2gnss_i = gnss_sat_pos_i - ant_pos_i;
    double normalizer = 1 / CalcNorm(ant2gnss_i);
    ant2gnss_i_n = normalizer * ant2gnss_i;

    // check gnss sats are visible from antenna
    double Re = environment::earth_equatorial_radius_m;
    double inner1 = InnerProduct(ant_pos_i, gnss_sat_pos_i);
    int is_visible_ant2gnss = 0;
    if (inner1 > 0)
      is_visible_ant2gnss = 1;
    else {
      Vector<3> tmp = ant_pos_i + InnerProduct(-ant_pos_i, ant2gnss_i_n) * ant2gnss_i;
      if (CalcNorm(tmp) < Re)
        // There is earth between antenna and gnss
        is_visible_ant2gnss = 0;
      else
        // There is not earth between antenna and gnss
        is_visible_ant2gnss = 1;
    }

    double inner2 = InnerProduct(antenna_direction_i, ant2gnss_i_n);
    if (inner2 > cos(half_width_ * libra::deg_to_rad) && is_visible_ant2gnss) {
      // is visible
      gnss_sats_visible_num_++;
      SetGnssInfo(ant2gnss_i, q_i2b, id_tmp);
    }
  }

  if (gnss_sats_visible_num_ > 0)
    is_gnss_sats_visible_ = 1;
  else
    is_gnss_sats_visible_ = 0;
}

void GNSSReceiver::SetGnssInfo(Vector<3> ant2gnss_i, Quaternion q_i2b, std::string gnss_id) {
  Vector<3> ant2gnss_b, ant2gnss_c;

  ant2gnss_b = q_i2b.FrameConversion(ant2gnss_i);
  ant2gnss_c = q_b2c_.FrameConversion(ant2gnss_b);

  double dist = CalcNorm(ant2gnss_c);
  double lon = AcTan(ant2gnss_c[1], ant2gnss_c[0]);
  double lat = AcTan(ant2gnss_c[2], sqrt(pow(ant2gnss_c[0], 2.0) + pow(ant2gnss_c[1], 2.0)));

  GnssInfo gnss_info_new = {gnss_id, lat, lon, dist};
  vec_gnssinfo_.push_back(gnss_info_new);
}

void GNSSReceiver::AddNoise(Vector<3> location_true_eci, Vector<3> location_true_ecef) {
  // Simplest noise model
  position_eci_[0] = location_true_eci[0] + nrs_eci_x_;
  position_eci_[1] = location_true_eci[1] + nrs_eci_y_;
  position_eci_[2] = location_true_eci[2] + nrs_eci_z_;

  // FIXME: noise in ECI frame is added to ECEF frame value
  position_ecef_[0] = location_true_ecef[0] + nrs_eci_x_;
  position_ecef_[1] = location_true_ecef[1] + nrs_eci_y_;
  position_ecef_[2] = location_true_ecef[2] + nrs_eci_z_;
}

void GNSSReceiver::ConvertJulianDayToGPSTime(const double JulianDay) {
  const double kJulianDayAtGPSTimeZero = 2444244.5;  // corresponds to 1980/1/5 midnight
  const double kDayInWeek = 7.0;
  // const double kSecInWeek = 604800.0;
  const double kSecInDay = 86400.0;

  // compute ToW from current JulianDay
  // note:"gpstime_week_" computed in this method is larger than 1024
  double elapsed_day = JulianDay - kJulianDayAtGPSTimeZero;
  gpstime_week_ = (unsigned int)(elapsed_day / kDayInWeek);
  gpstime_sec_ = (elapsed_day - (double)(gpstime_week_)*kDayInWeek) * kSecInDay;
}

std::string GNSSReceiver::GetLogHeader() const  // For logs
{
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(id_));
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

std::string GNSSReceiver::GetLogValue() const  // For logs
{
  std::string str_tmp = "";
  str_tmp += WriteScalar(utc_.year);
  str_tmp += WriteScalar(utc_.month);
  str_tmp += WriteScalar(utc_.day);
  str_tmp += WriteScalar(utc_.hour);
  str_tmp += WriteScalar(utc_.minute);
  str_tmp += WriteScalar(utc_.second);
  str_tmp += WriteVector(position_eci_, 10);
  str_tmp += WriteVector(velocity_ecef_, 10);
  str_tmp += WriteScalar(position_llh_[0], 10);
  str_tmp += WriteScalar(position_llh_[1], 10);
  str_tmp += WriteScalar(position_llh_[2], 10);
  str_tmp += WriteScalar(is_gnss_sats_visible_);
  str_tmp += WriteScalar(gnss_sats_visible_num_);

  return str_tmp;
}
