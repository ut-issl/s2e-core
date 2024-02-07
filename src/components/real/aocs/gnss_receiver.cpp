/**
 * @file gnss_receiver.cpp
 * @brief Class to emulate GNSS receiver
 */

#include "gnss_receiver.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/gnss/gnss_satellite_number.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/randomization/global_randomization.hpp>
#include <string>

GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, const size_t component_id, const std::string gnss_id,
                           const size_t max_channel, const AntennaModel antenna_model, const libra::Vector<3> antenna_position_b_m,
                           const libra::Quaternion quaternion_b2c, const double half_width_deg, const libra::Vector<3> noise_standard_deviation_m,
                           const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      max_channel_(max_channel),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      random_noise_i_x_(0.0, noise_standard_deviation_m[0], global_randomization.MakeSeed()),
      random_noise_i_y_(0.0, noise_standard_deviation_m[1], global_randomization.MakeSeed()),
      random_noise_i_z_(0.0, noise_standard_deviation_m[2], global_randomization.MakeSeed()),
      half_width_deg_(half_width_deg),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {}
GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
                           const std::string gnss_id, const size_t max_channel, const AntennaModel antenna_model,
                           const libra::Vector<3> antenna_position_b_m, const libra::Quaternion quaternion_b2c, const double half_width_deg,
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
      half_width_deg_(half_width_deg),
      gnss_id_(gnss_id),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {}

void GnssReceiver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  libra::Vector<3> position_true_eci_ = dynamics_->GetOrbit().GetPosition_i_m();
  libra::Quaternion quaternion_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();

  CheckAntenna(position_true_eci_, quaternion_i2b);

  if (is_gnss_visible_) {
    // Antenna of GNSS-R can detect GNSS signal
    position_ecef_m_ = dynamics_->GetOrbit().GetPosition_ecef_m();
    position_llh_ = dynamics_->GetOrbit().GetLatLonAlt();
    velocity_ecef_m_s_ = dynamics_->GetOrbit().GetVelocity_ecef_m_s();
    AddNoise(position_true_eci_, position_ecef_m_);
  } else {
    // position information will not be updated in this case
    utc_ = simulation_time_->GetCurrentUtc();
    ConvertJulianDayToGpsTime(simulation_time_->GetCurrentTime_jd());
  }
  // Time is updated with internal clock
  utc_ = simulation_time_->GetCurrentUtc();
  ConvertJulianDayToGpsTime(simulation_time_->GetCurrentTime_jd());
}

void GnssReceiver::CheckAntenna(const libra::Vector<3> position_true_eci_m, const libra::Quaternion quaternion_i2b) {
  if (antenna_model_ == AntennaModel::kSimple) {
    CheckAntennaSimple(position_true_eci_m, quaternion_i2b);
  } else if (antenna_model_ == AntennaModel::kCone) {
    CheckAntennaCone(position_true_eci_m, quaternion_i2b);
  } else {
    std::cout << "[Error] GNSS Receiver: Undefined antenna model." << std::endl;
  }
}

void GnssReceiver::CheckAntennaSimple(const libra::Vector<3> position_true_eci_m, const libra::Quaternion quaternion_i2b) {
  // Simplest model
  // GNSS satellites are visible when antenna directs anti-earth direction

  // Antenna normal vector at inertial frame
  libra::Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  libra::Vector<3> antenna_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_direction_c);
  libra::Vector<3> antenna_direction_i = quaternion_i2b.InverseFrameConversion(antenna_direction_b);

  double inner = InnerProduct(position_true_eci_m, antenna_direction_i);
  if (inner <= 0.0) {
    is_gnss_visible_ = false;
  } else {
    is_gnss_visible_ = true;
  }
}

void GnssReceiver::CheckAntennaCone(const libra::Vector<3> position_true_eci_m, const libra::Quaternion quaternion_i2b) {
  // Cone model
  gnss_information_list_.clear();

  // antenna normal vector at inertial frame
  libra::Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  libra::Vector<3> antenna_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_direction_c);
  libra::Vector<3> antenna_direction_i = quaternion_i2b.InverseFrameConversion(antenna_direction_b);

  libra::Vector<3> antenna_position_i_m = position_true_eci_m + quaternion_i2b.InverseFrameConversion(antenna_position_b_m_);

  // initialize
  visible_satellite_number_ = 0;

  for (size_t i = 0; i < kTotalNumberOfGnssSatellite; i++) {
    // check if gnss ID is compatible with the receiver
    std::string gnss_id_string = ConvertIndexToGnssSatelliteNumber(i);
    if (gnss_id_.find(gnss_id_string[0]) == std::string::npos) continue;

    // compute direction from sat to gnss in body-fixed frame
    libra::Vector<3> gnss_satellite_position_i_m = gnss_satellites_->GetPosition_eci_m(i);
    libra::Vector<3> antenna_to_gnss_satellite_i_m = gnss_satellite_position_i_m - antenna_position_i_m;
    libra::Vector<3> antenna_to_gnss_satellite_direction_i = antenna_to_gnss_satellite_i_m.CalcNormalizedVector();

    // check gnss satellites are visible from antenna
    double inner1 = InnerProduct(antenna_position_i_m, gnss_satellite_position_i_m);
    bool is_satellite_visible = false;
    if (inner1 > 0) {
      is_satellite_visible = true;
    } else {
      Vector<3> tmp =
          antenna_position_i_m + InnerProduct(-antenna_position_i_m, antenna_to_gnss_satellite_direction_i) * antenna_to_gnss_satellite_i_m;
      if (tmp.CalcNorm() < environment::earth_equatorial_radius_m) {
        // There is earth between antenna and gnss
        is_satellite_visible = false;
      } else {
        // There is not earth between antenna and gnss
        is_satellite_visible = true;
      }
    }

    double inner2 = InnerProduct(antenna_direction_i, antenna_to_gnss_satellite_direction_i);
    if (inner2 > cos(half_width_deg_ * libra::deg_to_rad) && is_satellite_visible) {
      // is visible
      visible_satellite_number_++;
      SetGnssInfo(antenna_to_gnss_satellite_i_m, quaternion_i2b, gnss_id_string);
    }
  }

  if (visible_satellite_number_ > 0) {
    is_gnss_visible_ = true;
  } else {
    is_gnss_visible_ = false;
  }
}

void GnssReceiver::SetGnssInfo(const libra::Vector<3> antenna_to_satellite_i_m, const libra::Quaternion quaternion_i2b, const std::string gnss_id) {
  libra::Vector<3> antenna_to_satellite_direction_b = quaternion_i2b.FrameConversion(antenna_to_satellite_i_m);
  libra::Vector<3> antenna_to_satellite_direction_c = quaternion_b2c_.FrameConversion(antenna_to_satellite_direction_b);

  double distance_m = antenna_to_satellite_i_m.CalcNorm();
  double longitude_rad = AcTan(antenna_to_satellite_direction_c[1], antenna_to_satellite_direction_c[0]);
  double latitude_rad =
      AcTan(antenna_to_satellite_direction_c[2], sqrt(pow(antenna_to_satellite_direction_c[0], 2.0) + pow(antenna_to_satellite_direction_c[1], 2.0)));

  GnssInfo gnss_info_new = {gnss_id, latitude_rad, longitude_rad, distance_m};
  gnss_information_list_.push_back(gnss_info_new);
}

void GnssReceiver::AddNoise(const libra::Vector<3> position_true_i_m, const libra::Vector<3> position_true_ecef_m) {
  // Simplest noise model
  position_eci_m_[0] = position_true_i_m[0] + random_noise_i_x_;
  position_eci_m_[1] = position_true_i_m[1] + random_noise_i_y_;
  position_eci_m_[2] = position_true_i_m[2] + random_noise_i_z_;

  // FIXME: noise in ECI frame is added to ECEF frame value
  position_ecef_m_[0] = position_true_ecef_m[0] + random_noise_i_x_;
  position_ecef_m_[1] = position_true_ecef_m[1] + random_noise_i_y_;
  position_ecef_m_[2] = position_true_ecef_m[2] + random_noise_i_z_;
}

void GnssReceiver::ConvertJulianDayToGpsTime(const double julian_day) {
  const double kJulianDayAtGpsTimeZero = 2444244.5;  // corresponds to 1980/1/5 midnight
  const double kDayInWeek = 7.0;
  const double kSecInDay = 86400.0;

  // compute time of week from current julian_day
  // note:"gps_time_week_ " computed in this method is larger than 1024
  double elapsed_day = julian_day - kJulianDayAtGpsTimeZero;
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

typedef struct _gnss_receiver_param {
  int prescaler;
  AntennaModel antenna_model;
  libra::Vector<3> antenna_pos_b;
  libra::Quaternion quaternion_b2c;
  double half_width_deg;
  std::string gnss_id;
  size_t max_channel;
  libra::Vector<3> noise_standard_deviation_m;
} GnssReceiverParam;

GnssReceiverParam ReadGnssReceiverIni(const std::string file_name, const GnssSatellites* gnss_satellites, const int component_id) {
  GnssReceiverParam gnss_receiver_param;

  IniAccess gnssr_conf(file_name);
  const char* sensor_name = "GNSS_RECEIVER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(component_id));
  const char* GSection = section_name.c_str();

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  gnss_receiver_param.prescaler = prescaler;

  gnss_receiver_param.antenna_model = static_cast<AntennaModel>(gnssr_conf.ReadInt(GSection, "antenna_model"));
  if (!gnss_satellites->IsCalcEnabled() && gnss_receiver_param.antenna_model == AntennaModel::kCone) {
    std::cout << "Calculation of GNSS SATELLITES is DISABLED, so the antenna "
                 "model of GNSS Receiver is automatically set to SIMPLE model."
              << std::endl;
    gnss_receiver_param.antenna_model = AntennaModel::kSimple;
  }

  gnssr_conf.ReadVector(GSection, "antenna_position_b_m", gnss_receiver_param.antenna_pos_b);
  gnssr_conf.ReadQuaternion(GSection, "quaternion_b2c", gnss_receiver_param.quaternion_b2c);
  gnss_receiver_param.half_width_deg = gnssr_conf.ReadDouble(GSection, "antenna_half_width_deg");
  gnss_receiver_param.gnss_id = gnssr_conf.ReadString(GSection, "gnss_id");
  gnss_receiver_param.max_channel = gnssr_conf.ReadInt(GSection, "maximum_channel");
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_eci_m", gnss_receiver_param.noise_standard_deviation_m);

  return gnss_receiver_param;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, const size_t component_id, const std::string file_name, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, component_id, gr_param.gnss_id, gr_param.max_channel, gr_param.antenna_model,
                      gr_param.antenna_pos_b, gr_param.quaternion_b2c, gr_param.half_width_deg, gr_param.noise_standard_deviation_m, dynamics,
                      gnss_satellites, simulation_time);
  return gnss_r;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id, const std::string file_name,
                              const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, power_port, component_id, gr_param.gnss_id, gr_param.max_channel, gr_param.antenna_model,
                      gr_param.antenna_pos_b, gr_param.quaternion_b2c, gr_param.half_width_deg, gr_param.noise_standard_deviation_m, dynamics,
                      gnss_satellites, simulation_time);
  return gnss_r;
}
