/**
 * @file sample_components.cpp
 * @brief An example of user side components management installed on a spacecraft
 */

#include "sample_components.hpp"

#include <library/initialize/initialize_file_access.hpp>

#include "sample_port_configuration.hpp"

SampleComponents::SampleComponents(const Dynamics* dynamics, Structure* structure, const LocalEnvironment* local_environment,
                                   const GlobalEnvironment* global_environment, const SimulationConfig* config, ClockGenerator* clock_gen,
                                   const int spacecraft_id)
    : config_(config), dynamics_(dynamics), structure_(structure), local_environment_(local_environment), glo_env_(global_environment) {
  IniAccess iniAccess = IniAccess(config_->spacecraft_file_list_[spacecraft_id]);

  // PCU power port connection
  pcu_ = new PowerControlUnit(clock_gen);
  pcu_->ConnectPort(0, 0.5, 3.3, 1.0);  // OBC: assumed power consumption is defined here
  pcu_->ConnectPort(1, 1.0);            // GyroSensor: assumed power consumption is defined inside the InitGyroSensor
  pcu_->ConnectPort(2, 1.0);            // for other all components

  // Components
  obc_ = new OnBoardComputer(1, clock_gen, pcu_->GetPowerPort(0));
  hils_port_manager_ = new HilsPortManager();

  // GyroSensor
  std::string ini_path = iniAccess.ReadString("COMPONENT_FILES", "gyro_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  gyro_ = new GyroSensor(
      InitGyroSensor(clock_gen, pcu_->GetPowerPort(1), 1, ini_path, glo_env_->GetSimulationTime().GetComponentStepTime_s(), dynamics_));

  // Magnetometer
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "magetometer_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  mag_sensor_ =
      new Magnetometer(InitMagetometer(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, glo_env_->GetSimulationTime().GetComponentStepTime_s(),
                                       &(local_environment_->GetGeomagneticField())));

  // StarSensor
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "stt_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  stt_ = new StarSensor(InitStarSensor(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, glo_env_->GetSimulationTime().GetComponentStepTime_s(),
                                       dynamics_, local_environment_));

  // SunSensor
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "ss_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  sun_sensor_ = new SunSensor(InitSunSensor(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, &(local_environment_->GetSolarRadiationPressure()),
                                            &(local_environment_->GetCelestialInformation())));

  // GNSS-R
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "gnss_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  gnss_ = new GnssReceiver(
      InitGnssReceiver(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, dynamics_, &(glo_env_->GetGnssSatellites()), &(glo_env_->GetSimulationTime())));

  // Magnetorquer
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "magetorquer_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  mag_torquer_ =
      new Magnetorquer(InitMagnetorquer(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, glo_env_->GetSimulationTime().GetComponentStepTime_s(),
                                        &(local_environment_->GetGeomagneticField())));

  // RW
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "rw_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  rw_ = new ReactionWheel(InitReactionWheel(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, dynamics_->GetAttitude().GetPropStep(),
                                            glo_env_->GetSimulationTime().GetComponentStepTime_s()));

  // Torque Generator
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "torque_generator_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  torque_generator_ = new TorqueGenerator(InitializeTorqueGenerator(clock_gen, ini_path, dynamics_));

  // Thruster
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "thruster_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  thruster_ = new SimpleThruster(InitSimpleThruster(clock_gen, pcu_->GetPowerPort(2), 1, ini_path, structure_, dynamics));

  // Force Generator
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "force_generator_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_gen, ini_path, dynamics_));

  // Antenna
  ini_path = iniAccess.ReadString("COMPONENT_FILES", "antenna_file");
  config_->main_logger_->CopyFileToLogDirectory(ini_path);
  antenna_ = new Antenna(InitAntenna(1, ini_path));

  // PCU power port initial control
  pcu_->GetPowerPort(0)->SetVoltage_V(3.3);
  pcu_->GetPowerPort(1)->SetVoltage_V(3.3);
  pcu_->GetPowerPort(2)->SetVoltage_V(3.3);

  /**  Examples  **/

  // ChangeStructure: Please uncomment change_structure related codes if you want to test the change_structure
  // change_structure_ = new ExampleChangeStructure(clock_gen, structure_);

  // UART tutorial. Comment out when not in use.
  // exp_hils_uart_responder_ = new ExampleSerialCommunicationForHils(clock_gen, 1, obc_, 3, 9600, hils_port_manager_, 1);
  // exp_hils_uart_sender_ = new ExampleSerialCommunicationForHils(clock_gen, 0, obc_, 4, 9600, hils_port_manager_, 0);

  // I2C tutorial. Comment out when not in use.
  // exp_hils_i2c_controller_ = new ExampleI2cControllerForHils(30, clock_gen, 5, 115200, 256, 256, hils_port_manager_);
  // exp_hils_i2c_target_ = new ExampleI2cTargetForHils(1, clock_gen, 0, 0x44, obc_, 6, hils_port_manager_);

  /**************/

  // actuator debug output
  // libra::Vector<kMtqDimension> mag_moment_c{0.01};
  // mag_torquer_->SetOutputMagneticMoment_c_Am2(mag_moment_c);
  // rw_->SetTargetTorque_rw_Nm(0.01);
  // rw_->SetDriveFlag(true);
  // thruster_->SetDuty(0.9);

  // force generator debug output
  // libra::Vector<3> force_N;
  // force_N[0] = 1.0;
  // force_N[1] = 0.0;
  // force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);

  // torque generator debug output
  // libra::Vector<3> torque_Nm;
  // torque_Nm[0] = 1.0;
  // torque_Nm[1] = 0.0;
  // torque_Nm[2] = 0.0;
  // torque_generator_->SetTorque_b_Nm(torque_Nm);
}

SampleComponents::~SampleComponents() {
  delete gyro_;
  delete mag_sensor_;
  delete stt_;
  delete sun_sensor_;
  delete gnss_;
  delete mag_torquer_;
  delete rw_;
  delete thruster_;
  delete force_generator_;
  delete torque_generator_;
  delete antenna_;
  // delete change_structure_;
  delete pcu_;
  // delete exp_hils_uart_responder_;
  // delete exp_hils_uart_sender_;
  // delete exp_hils_i2c_controller_;
  // delete exp_hils_i2c_target_;
  delete obc_;
  delete hils_port_manager_;  // delete after exp_hils
}

libra::Vector<3> SampleComponents::GenerateForce_b_N() {
  libra::Vector<3> force_b_N_(0.0);
  force_b_N_ += thruster_->GetOutputThrust_b_N();
  force_b_N_ += force_generator_->GetGeneratedForce_b_N();
  return force_b_N_;
}

libra::Vector<3> SampleComponents::GenerateTorque_b_Nm() {
  libra::Vector<3> torque_b_Nm_(0.0);
  torque_b_Nm_ += mag_torquer_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += rw_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += thruster_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += torque_generator_->GetGeneratedTorque_b_Nm();
  return torque_b_Nm_;
}

void SampleComponents::LogSetup(Logger& logger) {
  logger.AddLogList(gyro_);
  logger.AddLogList(mag_sensor_);
  logger.AddLogList(stt_);
  logger.AddLogList(sun_sensor_);
  logger.AddLogList(gnss_);
  logger.AddLogList(mag_torquer_);
  logger.AddLogList(rw_);
  logger.AddLogList(thruster_);
  logger.AddLogList(force_generator_);
  logger.AddLogList(torque_generator_);
}
