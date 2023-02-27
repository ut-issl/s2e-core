/**
 * @file initialize_gnss_receiver.cpp
 * @brief Initialize functions for GNSS Receiver
 */
#include "initialize_gnss_receiver.hpp"

#include <string.h>

#include "library/initialize/initialize_file_access.hpp"

typedef struct _gnssrecever_param {
  int prescaler;
  AntennaModel antenna_model;
  Vector<3> antenna_pos_b;
  Quaternion q_b2c;
  double half_width;
  std::string gnss_id;
  int ch_max;
  Vector<3> noise_std;
} GNSSReceiverParam;

GNSSReceiverParam ReadGNSSReceiverIni(const std::string fname, const GnssSatellites* gnss_satellites, const int id) {
  GNSSReceiverParam gnssreceiver_param;

  IniAccess gnssr_conf(fname);
  const char* sensor_name = "GNSS_RECEIVER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(id));
  const char* GSection = section_name.c_str();

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  gnssreceiver_param.prescaler = prescaler;

  gnssreceiver_param.antenna_model = static_cast<AntennaModel>(gnssr_conf.ReadInt(GSection, "antenna_model"));
  if (!gnss_satellites->IsCalcEnabled() && gnssreceiver_param.antenna_model == CONE) {
    std::cout << "Calculation of GNSS SATELLITES is DISABLED, so the antenna "
                 "model of GNSS Receiver is automatically set to SIMPLE model."
              << std::endl;
    gnssreceiver_param.antenna_model = SIMPLE;
  }

  gnssr_conf.ReadVector(GSection, "antenna_position_b_m", gnssreceiver_param.antenna_pos_b);
  gnssr_conf.ReadQuaternion(GSection, "quaternion_b2c", gnssreceiver_param.q_b2c);
  gnssreceiver_param.half_width = gnssr_conf.ReadDouble(GSection, "antenna_half_width_deg");
  gnssreceiver_param.gnss_id = gnssr_conf.ReadString(GSection, "gnss_id");
  gnssreceiver_param.ch_max = gnssr_conf.ReadInt(GSection, "maximum_channel");
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_eci_m", gnssreceiver_param.noise_std);

  return gnssreceiver_param;
}

GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_generator, int id, const std::string fname, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simtime) {
  GNSSReceiverParam gr_param = ReadGNSSReceiverIni(fname, gnss_satellites, id);

  GNSSReceiver gnss_r(gr_param.prescaler, clock_generator, id, gr_param.gnss_id, gr_param.ch_max, gr_param.antenna_model, gr_param.antenna_pos_b,
                      gr_param.q_b2c, gr_param.half_width, gr_param.noise_std, dynamics, gnss_satellites, simtime);
  return gnss_r;
}

GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_generator, PowerPort* power_port, int id, const std::string fname, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simtime) {
  GNSSReceiverParam gr_param = ReadGNSSReceiverIni(fname, gnss_satellites, id);

  // PowerPort
  power_port->InitializeWithInitializeFile(fname);

  GNSSReceiver gnss_r(gr_param.prescaler, clock_generator, power_port, id, gr_param.gnss_id, gr_param.ch_max, gr_param.antenna_model,
                      gr_param.antenna_pos_b, gr_param.q_b2c, gr_param.half_width, gr_param.noise_std, dynamics, gnss_satellites, simtime);
  return gnss_r;
}
