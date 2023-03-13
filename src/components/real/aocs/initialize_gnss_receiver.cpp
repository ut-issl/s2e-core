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
  libra::Vector<3> antenna_pos_b;
  libra::Quaternion quaternion_b2c;
  double half_width_rad;
  std::string gnss_id;
  int max_channel;
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
  gnss_receiver_param.half_width_rad = gnssr_conf.ReadDouble(GSection, "antenna_half_width_deg");
  gnss_receiver_param.gnss_id = gnssr_conf.ReadString(GSection, "gnss_id");
  gnss_receiver_param.max_channel = gnssr_conf.ReadInt(GSection, "maximum_channel");
  gnssr_conf.ReadVector(GSection, "white_noise_standard_deviation_eci_m", gnss_receiver_param.noise_standard_deviation_m);

  return gnss_receiver_param;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, int component_id, const std::string file_name, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, component_id, gr_param.gnss_id, gr_param.max_channel, gr_param.antenna_model,
                      gr_param.antenna_pos_b, gr_param.quaternion_b2c, gr_param.half_width_rad, gr_param.noise_standard_deviation_m, dynamics,
                      gnss_satellites, simulation_time);
  return gnss_r;
}

GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, PowerPort* power_port, int component_id, const std::string file_name,
                              const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time) {
  GnssReceiverParam gr_param = ReadGnssReceiverIni(file_name, gnss_satellites, component_id);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  GnssReceiver gnss_r(gr_param.prescaler, clock_generator, power_port, component_id, gr_param.gnss_id, gr_param.max_channel, gr_param.antenna_model,
                      gr_param.antenna_pos_b, gr_param.quaternion_b2c, gr_param.half_width_rad, gr_param.noise_standard_deviation_m, dynamics,
                      gnss_satellites, simulation_time);
  return gnss_r;
}
