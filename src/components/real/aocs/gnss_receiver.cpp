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

namespace s2e::components {

GnssReceiver::GnssReceiver(const int prescaler, environment::ClockGenerator* clock_generator, const size_t component_id,
                           const AntennaModel antenna_model, const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c,
                           const double half_width_deg, const math::Vector<4> klobuchar_alpha,
               const math::Vector<4> klobuchar_beta, const double pseudorange_noise_standard_deviation_m,
                           const math::Vector<3> position_noise_standard_deviation_ecef_m,
                           const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s, const bool is_log_pseudorange_enabled,
                           const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                           const environment::SimulationTime* simulation_time)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      half_width_deg_(half_width_deg),
      antenna_model_(antenna_model),
      klobuchar_alpha_(klobuchar_alpha),
      klobuchar_beta_(klobuchar_beta),
      is_logged_pseudorange_(is_log_pseudorange_enabled),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {
  for (size_t i = 0; i < 3; i++) {
    position_random_noise_ecef_m_[i].SetParameters(0.0, position_noise_standard_deviation_ecef_m[i], randomization::global_randomization.MakeSeed());
    velocity_random_noise_ecef_m_s_[i].SetParameters(0.0, velocity_noise_standard_deviation_ecef_m_s[i],
                                                     randomization::global_randomization.MakeSeed());
  }
  pseudorange_random_noise_m_.SetParameters(0.0, pseudorange_noise_standard_deviation_m, randomization::global_randomization.MakeSeed());
}

GnssReceiver::GnssReceiver(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
                           const AntennaModel antenna_model, const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c,
                           const double half_width_deg, const math::Vector<4> klobuchar_alpha,
               const math::Vector<4> klobuchar_beta, const double pseudorange_noise_standard_deviation_m,
                           const math::Vector<3> position_noise_standard_deviation_ecef_m,
                           const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s, const bool is_log_pseudorange_enabled,
                           const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                           const environment::SimulationTime* simulation_time)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      antenna_position_b_m_(antenna_position_b_m),
      quaternion_b2c_(quaternion_b2c),
      half_width_deg_(half_width_deg),
      antenna_model_(antenna_model),
      klobuchar_alpha_(klobuchar_alpha),
      klobuchar_beta_(klobuchar_beta),
      is_logged_pseudorange_(is_log_pseudorange_enabled),
      dynamics_(dynamics),
      gnss_satellites_(gnss_satellites),
      simulation_time_(simulation_time) {
  for (size_t i = 0; i < 3; i++) {
    position_random_noise_ecef_m_[i].SetParameters(0.0, position_noise_standard_deviation_ecef_m[i], randomization::global_randomization.MakeSeed());
    velocity_random_noise_ecef_m_s_[i].SetParameters(0.0, velocity_noise_standard_deviation_ecef_m_s[i],
                                                     randomization::global_randomization.MakeSeed());
  }
  pseudorange_random_noise_m_.SetParameters(0.0, pseudorange_noise_standard_deviation_m, randomization::global_randomization.MakeSeed());
}

void GnssReceiver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // Antenna checking
  // TODO: Use ECEF position only
  math::Vector<3> position_true_eci = dynamics_->GetOrbit().GetPosition_i_m();
  math::Quaternion quaternion_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  CheckAntenna(position_true_eci, quaternion_i2b);

  // Pseudorange calculation
  SetGnssObservationList();

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
    if (inner2 > cos(half_width_deg_ * math::deg_to_rad) && is_gnss_satellite_visible_from_receiver) {
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

double GnssReceiver::CalcGeometricDistance_m(const size_t gnss_system_id) {
  const double c_m_s = environment::speed_of_light_m_s;

  const math::Vector<3> receiver_position_true_ecef_m = dynamics_->GetOrbit().GetPosition_ecef_m();
  const math::Vector<3> gnss_position_at_signal_arrival_ecef_m = gnss_satellites_->GetPosition_ecef_m(gnss_system_id);

  const time_system::EpochTime current_epoch_time = simulation_time_->GetCurrentEpochTime();

  const double signal_travel_time_s = (gnss_position_at_signal_arrival_ecef_m - receiver_position_true_ecef_m).CalcNorm() / c_m_s;
  const time_system::EpochTime signal_travel_epoch_time(signal_travel_time_s);
  const time_system::EpochTime signal_emission_epoch_time = current_epoch_time - signal_travel_epoch_time;

  const math::Vector<3> gnss_position_at_signal_emission_ecef_m = gnss_satellites_->GetPosition_ecef_m(gnss_system_id, signal_emission_epoch_time);

  math::Vector<3> earth_angular_velocity_rad_s{0.0};
  earth_angular_velocity_rad_s[2] = environment::earth_mean_angular_velocity_rad_s;

  // Calculate Sagnac-effect correction
  const double sagnac_correction_m =
      InnerProduct(OuterProduct(gnss_position_at_signal_emission_ecef_m, receiver_position_true_ecef_m), earth_angular_velocity_rad_s) / c_m_s;

  const double geometric_distance_m = (gnss_position_at_signal_emission_ecef_m - receiver_position_true_ecef_m).CalcNorm() + sagnac_correction_m;
  return geometric_distance_m;
}

std::vector<double> GnssReceiver::CalcElevationAzimuth_rad(const size_t gnss_system_id) {
  const math::Vector<3> receiver_position_true_ecef_m = dynamics_->GetOrbit().GetPosition_ecef_m();
  const math::Vector<3> gnss_position_m = gnss_satellites_->GetPosition_ecef_m(gnss_system_id);

  math::Vector<3> receiver_to_gnss_direction_ecef = (gnss_position_m - receiver_position_true_ecef_m).CalcNormalizedVector();

  double receiver_latitude_rad = geodetic_position_.GetLatitude_rad();
  double receiver_longitude_rad = geodetic_position_.GetLongitude_rad();

  // Calculate unit vectors for North, East, and Up directions at the receiver's position
  math::Vector<3> unit_vector_north;
  unit_vector_north[0] = -sin(receiver_longitude_rad);
  unit_vector_north[1] = cos(receiver_longitude_rad);
  unit_vector_north[2] = 0.0;

  math::Vector<3> unit_vector_east;
  unit_vector_east[0] = -sin(receiver_latitude_rad) * cos(receiver_longitude_rad);
  unit_vector_east[1] = -sin(receiver_latitude_rad) * sin(receiver_longitude_rad);
  unit_vector_east[2] = cos(receiver_latitude_rad);

  math::Vector<3> unit_vector_up;
  unit_vector_up[0] = cos(receiver_latitude_rad) * cos(receiver_longitude_rad);
  unit_vector_up[1] = cos(receiver_latitude_rad) * sin(receiver_longitude_rad);
  unit_vector_up[2] = sin(receiver_latitude_rad);

  // Calculate elevation and azimuth angles
  double elevation_rad = asin(InnerProduct(receiver_to_gnss_direction_ecef, unit_vector_up));
  double azimuth_rad = atan2(InnerProduct(receiver_to_gnss_direction_ecef, unit_vector_east),
                             InnerProduct(receiver_to_gnss_direction_ecef, unit_vector_north));
  return {elevation_rad, azimuth_rad};
}

double GnssReceiver::CalcIonosphericDelay_m(const size_t gnss_system_id, const size_t band_id) {
  // Ref: Klobuchar, J.A., (1996) "Ionosphercic Effects on GPS", in Parkinson, Spilker (ed), "Global Positioning System Theory and Applications, pp.513-514.
  klobuchar_alpha_[0]=3.82E-8;
  klobuchar_alpha_[1]=1.49E-8;
  klobuchar_alpha_[2]=-1.79E-7;
  klobuchar_alpha_[3]=0.0;
  klobuchar_beta_[0]=1.43E5;
  klobuchar_beta_[1]=0.0;
  klobuchar_beta_[2]=-3.28E5;
  klobuchar_beta_[3]=1.13E5;
  const double rad2semi = 1.0 / math::pi; // Convert radians to semicircles
  const double semi2rad = math::pi; // Convert semicircles to radians

  std::vector<double> elevation_azimuth_rad = CalcElevationAzimuth_rad(gnss_system_id);
  double elevation_rad = elevation_azimuth_rad[0];
  double azimuth_rad = elevation_azimuth_rad[1];
  elevation_rad = 20.0 * math::deg_to_rad;
  azimuth_rad = 210.0 * math::deg_to_rad;

  double earth_centered_angle_semi = 0.0137 / (elevation_rad * rad2semi + 0.11) - 0.022;
  printf("psi: %f\n", earth_centered_angle_semi);

  // Calculate the latitude, longitude and geomagnetic latitude of the ionospheric pierce point (IPP)
  double receiver_latitude_rad = geodetic_position_.GetLatitude_rad();
  double receiver_longitude_rad = geodetic_position_.GetLongitude_rad();
  receiver_latitude_rad = 40.0 * math::deg_to_rad;
  receiver_longitude_rad = 260.0 * math::deg_to_rad;
  double latitude_ipp_semi = receiver_latitude_rad * rad2semi + earth_centered_angle_semi * cos(azimuth_rad);
  if (latitude_ipp_semi > 0.416) {
    latitude_ipp_semi = 0.416;
  } else if (latitude_ipp_semi < -0.416) {
    latitude_ipp_semi = -0.416;
  }
  printf("Phi_I: %f\n", latitude_ipp_semi);
  double longitude_ipp_semi = receiver_longitude_rad * rad2semi + earth_centered_angle_semi * sin(azimuth_rad) / cos(latitude_ipp_semi * semi2rad);
  printf("lambda_I: %f\n", longitude_ipp_semi);
  double geomagnetic_latitude_ipp_semi = latitude_ipp_semi + 0.064 * cos((longitude_ipp_semi - 1.617) * semi2rad);
  printf("Phi_m: %f\n", geomagnetic_latitude_ipp_semi);

  gps_time_s_ = 593100.0;
  double local_time_ipp_s = (43200.0 * longitude_ipp_semi) + gps_time_s_;
  local_time_ipp_s = fmod(local_time_ipp_s, 86400.0);
  if (local_time_ipp_s < 0.0) {
    local_time_ipp_s += 86400.0;
  }
  if (local_time_ipp_s > 86400.0) {
    local_time_ipp_s -= 86400.0;
  }
  printf("t: %f\n", local_time_ipp_s);

  double amplitude_s = klobuchar_alpha_[0] + klobuchar_alpha_[1] * geomagnetic_latitude_ipp_semi + klobuchar_alpha_[2] * pow(geomagnetic_latitude_ipp_semi, 2.0) +
                       klobuchar_alpha_[3] * pow(geomagnetic_latitude_ipp_semi, 3.0);
  if (amplitude_s < 0.0) {
    amplitude_s = 0.0;
  }

  double period_s = klobuchar_beta_[0] + klobuchar_beta_[1] * geomagnetic_latitude_ipp_semi + klobuchar_beta_[2] * pow(geomagnetic_latitude_ipp_semi, 2.0) +
                    klobuchar_beta_[3] * pow(geomagnetic_latitude_ipp_semi, 3.0);
  if (period_s < 72000.0) {
    period_s = 72000.0;
  }

  double phase_rad = (2.0 * math::pi / period_s) * (local_time_ipp_s - 50400.0);

  double slant_factor = 1.0 + 16.0 * pow(0.53 - elevation_rad * rad2semi, 3.0);
  printf("F: %f\n", slant_factor);

  double ionospheric_delay_gps_l1_m;
  if (abs(phase_rad) <= 1.57) {
    ionospheric_delay_gps_l1_m = slant_factor * (5.0E-09 + amplitude_s * (1.0 - pow(phase_rad, 2.0) / 2.0 + (pow(phase_rad, 4.0) / 24.0)));
  } else {
    ionospheric_delay_gps_l1_m = slant_factor * (5.0E-09 + amplitude_s);
  }
  printf("T_IONO: %.12f\n", ionospheric_delay_gps_l1_m);

  if (band_id == 1) {
    return ionospheric_delay_gps_l1_m;
  } else if (band_id == 2) {
    return pow((1.57542E9 / 1.22760E9), 2.0) * ionospheric_delay_gps_l1_m;
  } else if (band_id == 5) {
    return pow((1.57542E9 / 1.17645E9), 2.0) * ionospheric_delay_gps_l1_m;
  } else {
    std::cout << "[Error] GNSS Receiver: Undefined band ID for ionospheric delay calculation." << std::endl;
    return ionospheric_delay_gps_l1_m;
  }

}

double GnssReceiver::CalcPseudorange_m(const size_t gnss_system_id) {
  // TODO: Add effect of clock bias
  double geometric_distance_m = CalcGeometricDistance_m(gnss_system_id);
  double ionospheric_delay_m = CalcIonosphericDelay_m(gnss_system_id, 1);  // TODO: bandごとに計算できるように
  double pseudorange_m = geometric_distance_m + ionospheric_delay_m + pseudorange_random_noise_m_;
  return pseudorange_m;
}

void GnssReceiver::SetGnssObservationList() {
  // TODO: Add carrier phase observation
  pseudorange_list_m_.assign(kTotalNumberOfGnssSatellite, 0.0);
  for (size_t i = 0; i < gnss_information_list_.size(); i++) {
    size_t gnss_system_id = gnss_information_list_[i].gnss_id;
    double pseudorange_m = CalcPseudorange_m(gnss_system_id);
    pseudorange_list_m_[gnss_system_id] = pseudorange_m;
  }
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

typedef struct _gnss_receiver_param {
  int prescaler;
  AntennaModel antenna_model;
  math::Vector<3> antenna_pos_b;
  math::Quaternion quaternion_b2c;
  double half_width_deg;
  math::Vector<4> klobuchar_alpha;
  math::Vector<4> klobuchar_beta;
  double pseudorange_noise_standard_deviation_m;
  math::Vector<3> position_noise_standard_deviation_ecef_m;
  math::Vector<3> velocity_noise_standard_deviation_ecef_m_s;
  bool is_log_pseudorange_enabled;
} GnssReceiverParam;

std::string GnssReceiver::GetLogHeader() const  // For logs
{
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(component_id_));
  std::string sensor_name = "gnss_receiver" + sensor_id + "_";
  GnssReceiverParam gnss_receiver_param;

  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_year");
  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_month");
  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_day");
  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_hour");
  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_min");
  str_tmp += logger::WriteScalar(sensor_name + "measured_utc_time_sec");
  str_tmp += logger::WriteVector(sensor_name + "measured_position", "ecef", "m", 3);
  str_tmp += logger::WriteVector(sensor_name + "measured_velocity", "ecef", "m/s", 3);
  str_tmp += logger::WriteScalar(sensor_name + "measured_latitude", "rad");
  str_tmp += logger::WriteScalar(sensor_name + "measured_longitude", "rad");
  str_tmp += logger::WriteScalar(sensor_name + "measured_altitude", "m");
  str_tmp += logger::WriteScalar(sensor_name + "satellite_visible_flag");
  str_tmp += logger::WriteScalar(sensor_name + "number_of_visible_satellites");

  if (is_logged_pseudorange_) {
    for (size_t gps_index = 0; gps_index < kNumberOfGpsSatellite; gps_index++) {
      str_tmp += logger::WriteScalar("GPS" + std::to_string(gps_index) + "_pseudorange", "m");
    }
  }

  return str_tmp;
}

std::string GnssReceiver::GetLogValue() const  // For logs
{
  std::string str_tmp = "";
  GnssReceiverParam gnss_receiver_param;

  str_tmp += logger::WriteScalar(utc_.year);
  str_tmp += logger::WriteScalar(utc_.month);
  str_tmp += logger::WriteScalar(utc_.day);
  str_tmp += logger::WriteScalar(utc_.hour);
  str_tmp += logger::WriteScalar(utc_.minute);
  str_tmp += logger::WriteScalar(utc_.second);
  str_tmp += logger::WriteVector(position_ecef_m_, 10);
  str_tmp += logger::WriteVector(velocity_ecef_m_s_, 10);
  str_tmp += logger::WriteScalar(geodetic_position_.GetLatitude_rad(), 10);
  str_tmp += logger::WriteScalar(geodetic_position_.GetLongitude_rad(), 10);
  str_tmp += logger::WriteScalar(geodetic_position_.GetAltitude_m(), 10);
  str_tmp += logger::WriteScalar(is_gnss_visible_);
  str_tmp += logger::WriteScalar(visible_satellite_number_);

  if (is_logged_pseudorange_) {
    for (size_t gps_index = 0; gps_index < kNumberOfGpsSatellite; gps_index++) {
      str_tmp += logger::WriteScalar(pseudorange_list_m_[gps_index], 16);
    }
  }

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

GnssReceiverParam ReadGnssReceiverIni(const std::string file_name, const environment::GnssSatellites* gnss_satellites, const size_t component_id) {
  GnssReceiverParam gnss_receiver_param;

  setting_file_reader::IniAccess gnssr_conf(file_name);
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
  gnssr_conf.ReadVector(GSection, "klobuchar_alpha", gnss_receiver_param.klobuchar_alpha);
  gnssr_conf.ReadVector(GSection, "klobuchar_beta", gnss_receiver_param.klobuchar_beta);
  gnss_receiver_param.pseudorange_noise_standard_deviation_m = gnssr_conf.ReadDouble(GSection, "white_noise_standard_deviation_pseudorange_m");
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_position_ecef_m", gnss_receiver_param.position_noise_standard_deviation_ecef_m);
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_velocity_ecef_m_s", gnss_receiver_param.velocity_noise_standard_deviation_ecef_m_s);
  gnss_receiver_param.is_log_pseudorange_enabled = gnssr_conf.ReadEnable(GSection, "pseudorange_logging");
  return gnss_receiver_param;
}

GnssReceiver InitGnssReceiver(environment::ClockGenerator* clock_generator, const size_t component_id, const std::string file_name,
                              const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                              const environment::SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, component_id, gr_param.antenna_model, gr_param.antenna_pos_b, gr_param.quaternion_b2c,
                      gr_param.half_width_deg, gr_param.klobuchar_alpha, gr_param.klobuchar_beta, gr_param.pseudorange_noise_standard_deviation_m, gr_param.position_noise_standard_deviation_ecef_m,
                      gr_param.velocity_noise_standard_deviation_ecef_m_s, gr_param.is_log_pseudorange_enabled, dynamics, gnss_satellites,
                      simulation_time);
  return gnss_r;
}

GnssReceiver InitGnssReceiver(environment::ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
                              const std::string file_name, const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                              const environment::SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, power_port, component_id, gr_param.antenna_model, gr_param.antenna_pos_b,
                      gr_param.quaternion_b2c, gr_param.half_width_deg, gr_param.klobuchar_alpha, gr_param.klobuchar_beta, gr_param.pseudorange_noise_standard_deviation_m,
                      gr_param.position_noise_standard_deviation_ecef_m, gr_param.velocity_noise_standard_deviation_ecef_m_s,
                      gr_param.is_log_pseudorange_enabled, dynamics, gnss_satellites, simulation_time);
  return gnss_r;
}

}  // namespace s2e::components
