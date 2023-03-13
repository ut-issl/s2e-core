/*
 * @file initialize_antenna.cpp
 * @brief Initialize function for Antenna
 */

#define _CRT_SECURE_NO_WARNINGS
#include "initialize_antenna.hpp"

#include <string.h>

#include <library/math/vector.hpp>

#include "library/initialize/initialize_file_access.hpp"

using libra::Vector;

Antenna InitAntenna(const int antenna_id, const std::string file_name) {
  IniAccess antenna_conf(file_name);

  const std::string st_antenna_id = std::to_string(static_cast<long long>(antenna_id));
  const char *cs = st_antenna_id.data();

  char Section[30] = "ANTENNA_";
  strcat(Section, cs);

  Quaternion quaternion_b2c;
  antenna_conf.ReadQuaternion(Section, "quaternion_b2c", quaternion_b2c);

  bool is_transmitter = antenna_conf.ReadBoolean(Section, "is_transmitter");
  bool is_receiver = antenna_conf.ReadBoolean(Section, "is_receiver");
  double frequency_MHz = antenna_conf.ReadDouble(Section, "frequency_MHz");

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

  Antenna antenna(antenna_id, quaternion_b2c, is_transmitter, is_receiver, frequency_MHz, tx_output_power_W, tx_parameters,
                  rx_system_noise_temperature_K, rx_parameters);
  return antenna;
}
