#define _CRT_SECURE_NO_WARNINGS
#include <Component/CommGS/ANT.h>
#include <string.h>

#include <Library/math/Vector.hpp>

#include "../Initialize.h"
using libra::Vector;

// アンテナ初期化，ant_idで対応するアンテナ読み込み
ANT InitANT(int ant_id, const std::string fname) {
  IniAccess ant_conf(fname);

  const std::string st_ant_id = std::to_string(static_cast<long long>(ant_id));
  const char *cs = st_ant_id.data();

  char Section[30] = "ANT";
  strcat(Section, cs);

  Quaternion q_b2c;
  ant_conf.ReadQuaternion(Section, "q_b2c", q_b2c);

  bool is_transmitter = ant_conf.ReadBoolean(Section, "is_transmitter");
  bool is_receiver = ant_conf.ReadBoolean(Section, "is_receiver");
  double frequency = ant_conf.ReadDouble(Section, "frequency");

  Vector<4> tx_params;
  Vector<4> rx_params;
  if (is_transmitter) {
    tx_params[0] = ant_conf.ReadDouble(Section, "tx_output");
    tx_params[1] = ant_conf.ReadDouble(Section, "tx_gain");
    tx_params[2] = ant_conf.ReadDouble(Section, "tx_loss_feeder");
    tx_params[3] = ant_conf.ReadDouble(Section, "tx_loss_pointing");
  }
  if (is_receiver) {
    rx_params[0] = ant_conf.ReadDouble(Section, "rx_gain");
    rx_params[1] = ant_conf.ReadDouble(Section, "rx_loss_feeder");
    rx_params[2] = ant_conf.ReadDouble(Section, "rx_loss_pointing");
    rx_params[3] = ant_conf.ReadDouble(Section, "rx_system_noise_temperature");
  }

  ANT ant(ant_id, q_b2c, is_transmitter, is_receiver, frequency, tx_params, rx_params);
  return ant;
}
