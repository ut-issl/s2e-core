/**
 * @file sample_components.cpp
 * @brief An example of user side components management installed on a spacecraft
 */

#include "sample_components.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

#include "sample_port_configuration.hpp"

namespace s2e::sample {

using namespace components;

SampleComponents::SampleComponents(const dynamics::dynamics::Dynamics* dynamics, simulation::Structure* structure,
                                   const environment::LocalEnvironment* local_environment, const environment::GlobalEnvironment* global_environment,
                                   const simulation::SimulationConfiguration* configuration, environment::ClockGenerator* clock_generator,
                                   const unsigned int spacecraft_id)
    : configuration_(configuration),
      dynamics_(dynamics),
      structure_(structure),
      local_environment_(local_environment),
      global_environment_(global_environment) {
  setting_file_reader::IniAccess iniAccess = setting_file_reader::IniAccess(configuration_->spacecraft_file_list_[spacecraft_id]);

  // PCU power port connection
  pcu_ = new PowerControlUnit(clock_generator);
  pcu_->ConnectPort(0, 0.5, 3.3, 1.0);  // OBC: assumed power consumption is defined here
  pcu_->ConnectPort(1, 1.0);            // GyroSensor: assumed power consumption is defined inside the InitGyroSensor
  pcu_->ConnectPort(2, 1.0);            // for other all components

  // Components
  obc_ = new OnBoardComputer(1, clock_generator, pcu_->GetPowerPort(0));
  hils_port_manager_ = new simulation::HilsPortManager();

  // GyroSensor
  std::string file_name = iniAccess.ReadString("COMPONENT_FILES", "gyro_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  gyro_sensor_ = new GyroSensor(InitGyroSensor(clock_generator, pcu_->GetPowerPort(1), 1, file_name,
                                               global_environment_->GetSimulationTime().GetComponentStepTime_s(), dynamics_));

  // Magnetometer
  file_name = iniAccess.ReadString("COMPONENT_FILES", "magnetometer_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  magnetometer_ = new Magnetometer(InitMagnetometer(clock_generator, pcu_->GetPowerPort(2), 1, file_name,
                                                    global_environment_->GetSimulationTime().GetComponentStepTime_s(),
                                                    &(local_environment_->GetGeomagneticField())));

  // StarSensor
  file_name = iniAccess.ReadString("COMPONENT_FILES", "stt_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  star_sensor_ = new StarSensor(InitStarSensor(clock_generator, pcu_->GetPowerPort(2), 1, file_name,
                                               global_environment_->GetSimulationTime().GetComponentStepTime_s(), dynamics_, local_environment_));

  // SunSensor
  file_name = iniAccess.ReadString("COMPONENT_FILES", "ss_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  sun_sensor_ = new SunSensor(InitSunSensor(clock_generator, pcu_->GetPowerPort(2), 1, file_name, &(local_environment_->GetSolarRadiationPressure()),
                                            &(local_environment_->GetCelestialInformation())));

  // GNSS-R
  file_name = iniAccess.ReadString("COMPONENT_FILES", "gnss_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  gnss_receiver_ = new GnssReceiver(InitGnssReceiver(clock_generator, pcu_->GetPowerPort(2), 1, file_name, dynamics_,
                                                     &(global_environment_->GetGnssSatellites()), &(global_environment_->GetSimulationTime())));

  // Magnetorquer
  file_name = iniAccess.ReadString("COMPONENT_FILES", "magnetorquer_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  magnetorquer_ = new Magnetorquer(InitMagnetorquer(clock_generator, pcu_->GetPowerPort(2), 1, file_name,
                                                    global_environment_->GetSimulationTime().GetComponentStepTime_s(),
                                                    &(local_environment_->GetGeomagneticField())));

  // RW
  file_name = iniAccess.ReadString("COMPONENT_FILES", "rw_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  reaction_wheel_ = new ReactionWheel(
      InitReactionWheel(clock_generator, pcu_->GetPowerPort(2), 1, file_name, global_environment_->GetSimulationTime().GetComponentStepTime_s()));

  // Torque Generator
  file_name = iniAccess.ReadString("COMPONENT_FILES", "torque_generator_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  torque_generator_ = new TorqueGenerator(InitializeTorqueGenerator(clock_generator, file_name, dynamics_));

  // Thruster
  file_name = iniAccess.ReadString("COMPONENT_FILES", "thruster_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  thruster_ = new SimpleThruster(InitSimpleThruster(clock_generator, pcu_->GetPowerPort(2), 1, file_name, structure_, dynamics));

  // Mission
  file_name = iniAccess.ReadString("COMPONENT_FILES", "telescope_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  telescope_ = new Telescope(InitTelescope(clock_generator, 1, file_name, &(dynamics_->GetAttitude()), &(global_environment_->GetHipparcosCatalog()),
                                           &(local_environment_->GetCelestialInformation()), &(dynamics_->GetOrbit())));

  // Force Generator
  file_name = iniAccess.ReadString("COMPONENT_FILES", "force_generator_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_generator, file_name, dynamics_));

  // Angular Velocity Observer
  file_name = iniAccess.ReadString("COMPONENT_FILES", "angular_velocity_observer_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  angular_velocity_observer_ = new AngularVelocityObserver(InitializeAngularVelocityObserver(
      clock_generator, file_name, global_environment_->GetSimulationTime().GetComponentStepTime_s(), dynamics_->GetAttitude()));

  // Attitude Observer
  file_name = iniAccess.ReadString("COMPONENT_FILES", "attitude_observer_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  attitude_observer_ = new AttitudeObserver(InitializeAttitudeObserver(clock_generator, file_name, dynamics_->GetAttitude()));

  // Orbit Observer
  file_name = iniAccess.ReadString("COMPONENT_FILES", "orbit_observer_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  orbit_observer_ = new OrbitObserver(InitializeOrbitObserver(clock_generator, file_name, dynamics_->GetOrbit()));

  // Antenna
  file_name = iniAccess.ReadString("COMPONENT_FILES", "antenna_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  antenna_ = new Antenna(InitAntenna(1, file_name));

  // Component interference
  file_name = iniAccess.ReadString("COMPONENT_FILES", "component_interference_file");
  configuration_->main_logger_->CopyFileToLogDirectory(file_name);
  mtq_magnetometer_interference_ = new MtqMagnetometerInterference(file_name, *magnetometer_, *magnetorquer_);

  // PCU power port initial control
  pcu_->GetPowerPort(0)->SetVoltage_V(3.3);
  pcu_->GetPowerPort(1)->SetVoltage_V(3.3);
  pcu_->GetPowerPort(2)->SetVoltage_V(3.3);

  /**  Examples  **/

  // ChangeStructure: Please uncomment change_structure related codes if you want to test the change_structure
  // change_structure_ = new ExampleChangeStructure(clock_generator, structure_);

  // UART tutorial. Comment out when not in use.
  // exp_hils_uart_responder_ = new ExampleSerialCommunicationForHils(clock_generator, 1, obc_, 3, 9600, hils_port_manager_, 1);
  // exp_hils_uart_sender_ = new ExampleSerialCommunicationForHils(clock_generator, 0, obc_, 4, 9600, hils_port_manager_, 0);

  // I2C tutorial. Comment out when not in use.
  // exp_hils_i2c_controller_ = new ExampleI2cControllerForHils(30, clock_generator, 5, 115200, 256, 256, hils_port_manager_);
  // exp_hils_i2c_target_ = new ExampleI2cTargetForHils(1, clock_generator, 0, 0x44, obc_, 6, hils_port_manager_);

  /**************/

  // actuator debug output
  // math::Vector<kMtqDimension> mag_moment_c{0.01};
  // magnetorquer_->SetOutputMagneticMoment_c_Am2(mag_moment_c);
  // reaction_wheel_->SetTargetTorque_rw_Nm(0.01);
  // reaction_wheel_->SetDriveFlag(true);
  // thruster_->SetDuty(0.9);

  // force generator debug output
  // math::Vector<3> force_N;
  // force_N[0] = 1.0;
  // force_N[1] = 0.0;
  // force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);

  // torque generator debug output
  // math::Vector<3> torque_Nm;
  // torque_Nm[0] = 0.1;
  // torque_Nm[1] = 0.0;
  // torque_Nm[2] = 0.0;
  // torque_generator_->SetTorque_b_Nm(torque_Nm);
}

SampleComponents::~SampleComponents() {
  delete gyro_sensor_;
  delete magnetometer_;
  delete star_sensor_;
  delete sun_sensor_;
  delete gnss_receiver_;
  delete magnetorquer_;
  delete reaction_wheel_;
  delete thruster_;
  delete force_generator_;
  delete torque_generator_;
  delete angular_velocity_observer_;
  delete attitude_observer_;
  delete orbit_observer_;
  delete antenna_;
  delete mtq_magnetometer_interference_;
  // delete change_structure_;
  delete pcu_;
  // delete exp_hils_uart_responder_;
  // delete exp_hils_uart_sender_;
  // delete exp_hils_i2c_controller_;
  // delete exp_hils_i2c_target_;
  delete obc_;
  delete hils_port_manager_;  // delete after exp_hils
}

math::Vector<3> SampleComponents::GenerateForce_b_N() {
  math::Vector<3> force_b_N_(0.0);
  force_b_N_ += thruster_->GetOutputThrust_b_N();
  force_b_N_ += force_generator_->GetGeneratedForce_b_N();
  return force_b_N_;
}

math::Vector<3> SampleComponents::GenerateTorque_b_Nm() {
  math::Vector<3> torque_b_Nm_(0.0);
  torque_b_Nm_ += magnetorquer_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += reaction_wheel_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += thruster_->GetOutputTorque_b_Nm();
  torque_b_Nm_ += torque_generator_->GetGeneratedTorque_b_Nm();
  return torque_b_Nm_;
}

void SampleComponents::ComponentInterference() { mtq_magnetometer_interference_->UpdateInterference(); }

void SampleComponents::LogSetup(logger::Logger& logger) {
  logger.AddLogList(gyro_sensor_);
  logger.AddLogList(magnetometer_);
  logger.AddLogList(star_sensor_);
  logger.AddLogList(sun_sensor_);
  logger.AddLogList(gnss_receiver_);
  logger.AddLogList(magnetorquer_);
  logger.AddLogList(reaction_wheel_);
  logger.AddLogList(thruster_);
  logger.AddLogList(telescope_);
  logger.AddLogList(force_generator_);
  logger.AddLogList(torque_generator_);
  logger.AddLogList(angular_velocity_observer_);
  logger.AddLogList(attitude_observer_);
  logger.AddLogList(orbit_observer_);
}

}  // namespace s2e::sample
