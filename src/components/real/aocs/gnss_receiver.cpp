/**
 * @file gnss_receiver.cpp
 * @brief Class to emulate GNSS receiver
 */

#include "gnss_receiver.hpp"

#include <environment/global/physical_constants.hpp>
#include <math_physics/gnss/gnss_satellite_number.hpp>
#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>
#include <string>

GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, const size_t component_id, const AntennaModel antenna_model,
                           const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c, const double half_width_deg,
                           const math::Vector<3> position_noise_standard_deviation_ecef_m,
                           const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s, const Dynamics* dynamics,
                           const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      half_width_deg_(half_width_deg),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {
  for (size_t i = 0; i < 3; i++) {
    position_random_noise_ecef_m_[i].SetParameters(0.0, position_noise_standard_deviation_ecef_m[i], global_randomization.MakeSeed());
    velocity_random_noise_ecef_m_s_[i].SetParameters(0.0, velocity_noise_standard_deviation_ecef_m_s[i], global_randomization.MakeSeed());
  }
}

GnssReceiver::GnssReceiver(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
                           const AntennaModel antenna_model, const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c,
                           const double half_width_deg, const math::Vector<3> position_noise_standard_deviation_ecef_m,
                           const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s, const Dynamics* dynamics,
                           const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      half_width_deg_(half_width_deg),
      antenna_model_(antenna_model),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {
  for (size_t i = 0; i < 3; i++) {
    position_random_noise_ecef_m_[i].SetParameters(0.0, position_noise_standard_deviation_ecef_m[i], global_randomization.MakeSeed());
    velocity_random_noise_ecef_m_s_[i].SetParameters(0.0, velocity_noise_standard_deviation_ecef_m_s[i], global_randomization.MakeSeed());
  }
}

void GnssReceiver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // Antenna checking
  // TODO: Use ECEF position only
  math::Vector<3> position_true_eci = dynamics_->GetOrbit().GetPosition_i_m();
  math::Quaternion quaternion_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  CheckAntenna(position_true_eci, quaternion_i2b);

  if (is_gnss_visible_) {
    // Antenna of GNSS-R can detect GNSS signal
    position_ecef_m_ = dynamics_->GetOrbit().GetPosition_ecef_m();
    velocity_ecef_m_s_ = dynamics_->GetOrbit().GetVelocity_ecef_m_s();
    AddNoise(position_ecef_m_, velocity_ecef_m_s_);
    // Convert observed value to other frames
    geodetic_position_.UpdateFromEcef(position_ecef_m_);
  }

  // Time is updated with internal clock
  utc_ = simulation_time_->GetCurrentUtc();
  ConvertJulianDayToGpsTime(simulation_time_->GetCurrentTime_jd());
}

void GnssReceiver::CheckAntenna(const math::Vector<3> position_true_eci_m, const math::Quaternion quaternion_i2b) {
  if (antenna_model_ == AntennaModel::kSimple) {
    CheckAntennaSimple(position_true_eci_m, quaternion_i2b);
  } else if (antenna_model_ == AntennaModel::kCone) {
    CheckAntennaCone(position_true_eci_m, quaternion_i2b);
  } else {
    std::cout << "[Error] GNSS Receiver: Undefined antenna model." << std::endl;
  }
}

void GnssReceiver::CheckAntennaSimple(const math::Vector<3> position_true_eci_m, const math::Quaternion quaternion_i2b) {
  // Simplest model
  // GNSS satellites are visible when antenna directs anti-earth direction

  // Antenna normal vector at inertial frame
  math::Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  math::Vector<3> antenna_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_direction_c);
  math::Vector<3> antenna_direction_i = quaternion_i2b.InverseFrameConversion(antenna_direction_b);

  double inner = InnerProduct(position_true_eci_m, antenna_direction_i);
  if (inner <= 0.0) {
    is_gnss_visible_ = false;
  } else {
    is_gnss_visible_ = true;
  }
}

void GnssReceiver::CheckAntennaCone(const math::Vector<3> position_true_eci_m, const math::Quaternion quaternion_i2b) {
  // Cone model
  gnss_information_list_.clear();

  // Antenna pointing direction vector at inertial frame
  math::Vector<3> antenna_pointing_direction_c(0.0);
  antenna_pointing_direction_c[2] = 1.0;
  math::Vector<3> antenna_pointing_direction_b = quaternion_b2c_.InverseFrameConversion(antenna_pointing_direction_c);
  math::Vector<3> antenna_pointing_direction_i = quaternion_i2b.InverseFrameConversion(antenna_pointing_direction_b);

  // Antenna position vector at inertial frame
  math::Vector<3> antenna_position_i_m = position_true_eci_m + quaternion_i2b.InverseFrameConversion(antenna_position_b_m_);

  // initialize
  visible_satellite_number_ = 0;

  size_t number_of_calculated_gnss_satellites = gnss_satellites_->GetNumberOfCalculatedSatellite();

  for (size_t i = 0; i < number_of_calculated_gnss_satellites; i++) {
    // compute direction from sat to gnss in body-fixed frame
    math::Vector<3> gnss_satellite_position_i_m = gnss_satellites_->GetPosition_eci_m(i);
    math::Vector<3> antenna_to_gnss_satellite_i_m = gnss_satellite_position_i_m - antenna_position_i_m;
    math::Vector<3> antenna_to_gnss_satellite_direction_i = antenna_to_gnss_satellite_i_m.CalcNormalizedVector();

    // Check GNSS satellites are visible from the receiver(not care antenna direction)
    bool is_gnss_satellite_visible_from_receiver = false;
    double inner1 = InnerProduct(antenna_position_i_m, gnss_satellite_position_i_m);
    if (inner1 > 0.0) {  // GNSS satellite and receiver are in the same hemisphere
      is_gnss_satellite_visible_from_receiver = true;
    } else {  // GNSS satellite is in the another hemisphere
      double angle_bw_earth_center_and_edge_rad = asin(environment::earth_equatorial_radius_m / antenna_position_i_m.CalcNorm());
      double angle_bw_earth_center_and_gnss_rad =
          acos(InnerProduct(-antenna_position_i_m.CalcNormalizedVector(), antenna_to_gnss_satellite_direction_i));

      if (angle_bw_earth_center_and_edge_rad < angle_bw_earth_center_and_gnss_rad) {
        // There is no Earth between receiver and GNSS satellite
        is_gnss_satellite_visible_from_receiver = true;
      } else {
        // There is Earth between receiver and GNSS satellite
        is_gnss_satellite_visible_from_receiver = false;
      }
    }

    // Check GNSS satellites are in the antenna half width angle
    double inner2 = InnerProduct(antenna_pointing_direction_i, antenna_to_gnss_satellite_direction_i);
    if (inner2 > cos(half_width_deg_ * libra::deg_to_rad) && is_gnss_satellite_visible_from_receiver) {
      // is visible
      visible_satellite_number_++;
      SetGnssInfo(antenna_to_gnss_satellite_i_m, quaternion_i2b, i);
    }
  }

  if (visible_satellite_number_ >= 4) {
    is_gnss_visible_ = true;
  } else {
    is_gnss_visible_ = false;
  }
}

void GnssReceiver::SetGnssInfo(const math::Vector<3> antenna_to_satellite_i_m, const math::Quaternion quaternion_i2b,
                               const std::size_t gnss_system_id) {
  math::Vector<3> antenna_to_satellite_direction_b = quaternion_i2b.FrameConversion(antenna_to_satellite_i_m);
  math::Vector<3> antenna_to_satellite_direction_c = quaternion_b2c_.FrameConversion(antenna_to_satellite_direction_b);

  double distance_m = antenna_to_satellite_i_m.CalcNorm();
  double longitude_rad = AcTan(antenna_to_satellite_direction_c[1], antenna_to_satellite_direction_c[0]);
  double latitude_rad =
      AcTan(antenna_to_satellite_direction_c[2], sqrt(pow(antenna_to_satellite_direction_c[0], 2.0) + pow(antenna_to_satellite_direction_c[1], 2.0)));

  GnssInfo gnss_info_new = {gnss_system_id, latitude_rad, longitude_rad, distance_m};
  gnss_information_list_.push_back(gnss_info_new);
}

void GnssReceiver::AddNoise(const math::Vector<3> position_true_ecef_m, const math::Vector<3> velocity_true_ecef_m_s) {
  for (size_t i = 0; i < 3; i++) {
    position_ecef_m_[i] = position_true_ecef_m[i] + position_random_noise_ecef_m_[i];
    velocity_ecef_m_s_[i] = velocity_true_ecef_m_s[i] + velocity_random_noise_ecef_m_s_[i];
  }
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
  str_tmp += WriteVector(sensor_name + "measured_position", "ecef", "m", 3);
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
  str_tmp += WriteVector(position_ecef_m_, 10);
  str_tmp += WriteVector(velocity_ecef_m_s_, 10);
  str_tmp += WriteScalar(geodetic_position_.GetLatitude_rad(), 10);
  str_tmp += WriteScalar(geodetic_position_.GetLongitude_rad(), 10);
  str_tmp += WriteScalar(geodetic_position_.GetAltitude_m(), 10);
  str_tmp += WriteScalar(is_gnss_visible_);
  str_tmp += WriteScalar(visible_satellite_number_);

  return str_tmp;
}

AntennaModel SetAntennaModel(const std::string antenna_model) {
  if (antenna_model == "SIMPLE") {
    return AntennaModel ::kSimple;
  } else if (antenna_model == "CONE") {
    return AntennaModel ::kCone;
  } else {
    std::cerr << "[WARNINGS] GNSS receiver antenna model is not defined!" << std::endl;
    std::cerr << "The antenna model is automatically initialized as SIMPLE mode" << std::endl;
    return AntennaModel ::kSimple;
  }
}

typedef struct _gnss_receiver_param {
  int prescaler;
  AntennaModel antenna_model;
  math::Vector<3> antenna_pos_b;
  math::Quaternion quaternion_b2c;
  double half_width_deg;
  math::Vector<3> position_noise_standard_deviation_ecef_m;
  math::Vector<3> velocity_noise_standard_deviation_ecef_m_s;
} GnssReceiverParam;

GnssReceiverParam ReadGnssReceiverIni(const std::string file_name, const GnssSatellites* gnss_satellites, const size_t component_id) {
  GnssReceiverParam gnss_receiver_param;

  IniAccess gnssr_conf(file_name);
  const char* sensor_name = "GNSS_RECEIVER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(component_id));
  const char* GSection = section_name.c_str();

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  gnss_receiver_param.prescaler = prescaler;

  std::string antenna_model_name = gnssr_conf.ReadString(GSection, "antenna_model");
  gnss_receiver_param.antenna_model = SetAntennaModel(antenna_model_name);
  if (!gnss_satellites->IsCalcEnabled() && gnss_receiver_param.antenna_model == AntennaModel::kCone) {
    std::cout << "[WARNINGS] Calculation of GNSS SATELLITES is DISABLED, "
                 "so the antenna model of GnssReceiver is automatically set to SIMPLE model."
              << std::endl;
    gnss_receiver_param.antenna_model = AntennaModel::kSimple;
  }

  gnssr_conf.ReadVector(GSection, "antenna_position_b_m", gnss_receiver_param.antenna_pos_b);
  gnssr_conf.ReadQuaternion(GSection, "quaternion_b2c", gnss_receiver_param.quaternion_b2c);
  gnss_receiver_param.half_width_deg = gnssr_conf.ReadDouble(GSection, "antenna_half_width_deg");
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_position_ecef_m", gnss_receiver_param.position_noise_standard_deviation_ecef_m);
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_velocity_ecef_m_s", gnss_receiver_param.velocity_noise_standard_deviation_ecef_m_s);

  return gnss_receiver_param;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, const size_t component_id, const std::string file_name, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, component_id, gr_param.antenna_model, gr_param.antenna_pos_b, gr_param.quaternion_b2c,
                      gr_param.half_width_deg, gr_param.position_noise_standard_deviation_ecef_m, gr_param.velocity_noise_standard_deviation_ecef_m_s,
                      dynamics, gnss_satellites, simulation_time);
  return gnss_r;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id, const std::string file_name,
                              const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, power_port, component_id, gr_param.antenna_model, gr_param.antenna_pos_b,
                      gr_param.quaternion_b2c, gr_param.half_width_deg, gr_param.position_noise_standard_deviation_ecef_m,
                      gr_param.velocity_noise_standard_deviation_ecef_m_s, dynamics, gnss_satellites, simulation_time);
  return gnss_r;
}
