/*
 * @file InitAntenna.cpp
 * @brief Initialize function for Antenna
 */

#define _CRT_SECURE_NO_WARNINGS
#include "InitAntenna.hpp"

#include <string.h>

#include <Library/math/Vector.hpp>

#include "Interface/InitInput/IniAccess.h"

using libra::Vector;

Antenna InitAntenna(const int antenna_id, const std::string file_name) {
  IniAccess antenna_conf(file_name);

  const std::string st_ant_id = std::to_string(static_cast<long long>(antenna_id));
  const char *cs = st_ant_id.data();

  char Section[30] = "ANT";
  strcat(Section, cs);

  Quaternion q_b2c;
  antenna_conf.ReadQuaternion(Section, "q_b2c", q_b2c);

  bool is_transmitter = antenna_conf.ReadBoolean(Section, "is_transmitter");
  bool is_receiver = antenna_conf.ReadBoolean(Section, "is_receiver");
  double frequency = antenna_conf.ReadDouble(Section, "frequency");

  Vector<4> tx_params;
  Vector<4> rx_params;
  if (is_transmitter) {
    tx_params[0] = antenna_conf.ReadDouble(Section, "tx_output");
    tx_params[1] = antenna_conf.ReadDouble(Section, "tx_gain");
    tx_params[2] = antenna_conf.ReadDouble(Section, "tx_loss_feeder");
    tx_params[3] = antenna_conf.ReadDouble(Section, "tx_loss_pointing");
  }
  if (is_receiver) {
    rx_params[0] = antenna_conf.ReadDouble(Section, "rx_gain");
    rx_params[1] = antenna_conf.ReadDouble(Section, "rx_loss_feeder");
    rx_params[2] = antenna_conf.ReadDouble(Section, "rx_loss_pointing");
    rx_params[3] = antenna_conf.ReadDouble(Section, "rx_system_noise_temperature");
  }

  Antenna antenna(antenna_id, q_b2c, is_transmitter, is_receiver, frequency, tx_params, rx_params);
  return antenna;
}
