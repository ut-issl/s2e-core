/*
 * @file antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "antenna.hpp"

#define _CRT_SECURE_NO_WARNINGS
#include <string.h>

#include <cmath>
#include <library/initialize/initialize_file_access.hpp>
#include <library/utilities/macros.hpp>

Antenna::Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
                 const double frequency_MHz, const Vector<5> tx_parameters, const Vector<4> rx_parameters)
    : component_id_(component_id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_MHz_(frequency_MHz) {
  quaternion_b2c_ = quaternion_b2c;

  // Parameters
  tx_output_power_W_ = tx_parameters[0];
  tx_parameters_.gain_dBi_ = tx_parameters[1];
  tx_parameters_.loss_feeder_dB_ = tx_parameters[2];
  tx_parameters_.loss_pointing_dB_ = tx_parameters[3];
  tx_bitrate_bps_ = tx_parameters[4];

  rx_parameters_.gain_dBi_ = rx_parameters[0];
  rx_parameters_.loss_feeder_dB_ = rx_parameters[1];
  rx_parameters_.loss_pointing_dB_ = rx_parameters[2];
  rx_system_noise_temperature_K_ = rx_parameters[3];

  // Antenna gain
  tx_parameters_.antenna_gain_model = AntennaGainModel::kIsotropic;
  rx_parameters_.antenna_gain_model = AntennaGainModel::kIsotropic;

  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_parameters_.loss_feeder_dB_ + tx_parameters_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_parameters_.loss_feeder_dB_ + rx_parameters_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
                 const double frequency_MHz, const double tx_bitrate_bps, const double tx_output_power_W, const AntennaParameters tx_parameters,
                 const double rx_system_noise_temperature_K, const AntennaParameters rx_parameters)
    : component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      is_transmitter_(is_transmitter),
      is_receiver_(is_receiver),
      frequency_MHz_(frequency_MHz),
      tx_bitrate_bps_(tx_bitrate_bps),
      tx_output_power_W_(tx_output_power_W),
      tx_parameters_(tx_parameters),
      rx_system_noise_temperature_K_(rx_system_noise_temperature_K),
      rx_parameters_(rx_parameters) {
  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_parameters_.loss_feeder_dB_ + tx_parameters_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_parameters_.loss_feeder_dB_ + rx_parameters_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::~Antenna() {}

double Antenna::CalcAntennaGain(const AntennaParameters antenna_parameters, const double theta_rad, const double phi_rad) const {
  double gain_dBi = 0.0;
  switch (antenna_parameters.antenna_gain_model) {
    case AntennaGainModel::kIsotropic:
      gain_dBi = antenna_parameters.gain_dBi_;
      break;
    case AntennaGainModel::kRadiationPatternCsv:
      gain_dBi = antenna_parameters.radiation_pattern.GetGain_dBi(theta_rad, phi_rad);
      break;
    default:
      break;
  }
  return gain_dBi;
}

double Antenna::CalcTxEirp_dBW(const double theta_rad, const double phi_rad) const {
  return tx_eirp_dBW_ + CalcAntennaGain(tx_parameters_, theta_rad, phi_rad);
}
double Antenna::CalcRxGt_dB_K(const double theta_rad, const double phi_rad) const {
  return rx_gt_dBK_ + CalcAntennaGain(rx_parameters_, theta_rad, phi_rad);
}

AntennaGainModel SetAntennaGainModel(const std::string gain_model_name) {
  if (gain_model_name == "ISOTROPIC") {
    return AntennaGainModel::kIsotropic;
  } else if (gain_model_name == "RADIATION_PATTERN_CSV") {
    return AntennaGainModel::kRadiationPatternCsv;
  } else {
    return AntennaGainModel::kIsotropic;
  }
}

Antenna InitAntenna(const int antenna_id, const std::string file_name) {
  IniAccess antenna_conf(file_name);

  const std::string st_antenna_id = std::to_string(static_cast<long long>(antenna_id));
  const char* cs = st_antenna_id.data();

  char Section[30] = "ANTENNA_";
  strcat(Section, cs);

  Quaternion quaternion_b2c;
  antenna_conf.ReadQuaternion(Section, "quaternion_b2c", quaternion_b2c);

  bool is_transmitter = antenna_conf.ReadBoolean(Section, "is_transmitter");
  bool is_receiver = antenna_conf.ReadBoolean(Section, "is_receiver");
  double frequency_MHz = antenna_conf.ReadDouble(Section, "frequency_MHz");

  double tx_bitrate_bps = antenna_conf.ReadDouble(Section, "tx_bitrate_bps");
  double tx_output_power_W = antenna_conf.ReadDouble(Section, "tx_output_W");
  double rx_system_noise_temperature_K = antenna_conf.ReadDouble(Section, "rx_system_noise_temperature_K");

  AntennaParameters tx_parameters;
  if (is_transmitter) {
    tx_parameters.gain_dBi_ = antenna_conf.ReadDouble(Section, "tx_gain_dBi");
    tx_parameters.loss_feeder_dB_ = antenna_conf.ReadDouble(Section, "tx_loss_feeder_dB");
    tx_parameters.loss_pointing_dB_ = antenna_conf.ReadDouble(Section, "tx_loss_pointing_dB");
    tx_parameters.antenna_gain_model = SetAntennaGainModel(antenna_conf.ReadString(Section, "tx_antenna_gain_model"));
    size_t length_theta = antenna_conf.ReadInt(Section, "tx_length_theta");
    size_t length_phi = antenna_conf.ReadInt(Section, "tx_length_phi");
    double theta_max_rad = antenna_conf.ReadDouble(Section, "tx_theta_max_rad");
    double phi_max_rad = antenna_conf.ReadDouble(Section, "tx_phi_max_rad");
    tx_parameters.radiation_pattern = AntennaRadiationPattern(antenna_conf.ReadString(Section, "tx_antenna_radiation_pattern_file"), length_theta,
                                                              length_phi, theta_max_rad, phi_max_rad);
  } else {
    tx_parameters.gain_dBi_ = 0.0;
    tx_parameters.loss_feeder_dB_ = 0.0;
    tx_parameters.loss_pointing_dB_ = 0.0;
    tx_parameters.antenna_gain_model = AntennaGainModel::kIsotropic;
  }

  AntennaParameters rx_parameters;
  if (is_receiver) {
    rx_parameters.gain_dBi_ = antenna_conf.ReadDouble(Section, "rx_gain_dBi");
    rx_parameters.loss_feeder_dB_ = antenna_conf.ReadDouble(Section, "rx_loss_feeder_dB");
    rx_parameters.loss_pointing_dB_ = antenna_conf.ReadDouble(Section, "rx_loss_pointing_dB");
    rx_parameters.antenna_gain_model = SetAntennaGainModel(antenna_conf.ReadString(Section, "rx_antenna_gain_model"));
    rx_parameters.radiation_pattern = AntennaRadiationPattern(antenna_conf.ReadString(Section, "rx_antenna_radiation_pattern_file"));
    size_t length_theta = antenna_conf.ReadInt(Section, "rx_length_theta");
    size_t length_phi = antenna_conf.ReadInt(Section, "rx_length_phi");
    double theta_max_rad = antenna_conf.ReadDouble(Section, "rx_theta_max_rad");
    double phi_max_rad = antenna_conf.ReadDouble(Section, "rx_phi_max_rad");
    rx_parameters.radiation_pattern = AntennaRadiationPattern(antenna_conf.ReadString(Section, "rx_antenna_radiation_pattern_file"), length_theta,
                                                              length_phi, theta_max_rad, phi_max_rad);
  } else {
    rx_parameters.gain_dBi_ = 0.0;
    rx_parameters.loss_feeder_dB_ = 0.0;
    rx_parameters.loss_pointing_dB_ = 0.0;
    rx_parameters.antenna_gain_model = AntennaGainModel::kIsotropic;
  }

  Antenna antenna(antenna_id, quaternion_b2c, is_transmitter, is_receiver, frequency_MHz, tx_bitrate_bps, tx_output_power_W, tx_parameters,
                  rx_system_noise_temperature_K, rx_parameters);
  return antenna;
}
