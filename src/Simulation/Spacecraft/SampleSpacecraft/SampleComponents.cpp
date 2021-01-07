#include "SampleComponents.h"
#include "Sample_PortConfig.h"

//TODO: Create a base class?
SampleComponents::SampleComponents(const Dynamics* dynamics, const Structure* structure, const SimulationConfig* config, ClockGenerator* clock_gen, const int sat_id)
  :dynamics_(dynamics), structure_(structure), config_(config)
{
  IniAccess iniAccess = IniAccess(config_->sat_file_[sat_id]);
  // PCU power port connection
  pcu_ = new PCU(clock_gen);
  pcu_->ConnectPort(0, 0.5, 3.3, 1.0);  // OBC: assumed power consumption is defined here
  pcu_->ConnectPort(1, 1.0);  // Gyro: assumed power consumption is defined inside the InitGyro

  // Components
  obc_ = new OBC(1, clock_gen, pcu_->GetPowerPort(0));

  string gyro_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_file");
  config_->main_logger_->CopyFileToLogDir(gyro_ini_path);
  gyro_ = new Gyro(InitGyro(clock_gen, pcu_->GetPowerPort(1), 1, gyro_ini_path, dynamics_));

  // PCU power port initial control
  pcu_->GetPowerPort(0)->SetVoltage(3.3);
  pcu_->GetPowerPort(1)->SetVoltage(3.3);
}

SampleComponents::~SampleComponents()
{
  delete gyro_;
  delete pcu_;
  delete obc_;
}

Vector<3> SampleComponents::GenerateForce_b()
{
  //There is no orbit control component, so it remains 0
  Vector<3> force_b_(0.0);
  return force_b_;
};

Vector<3> SampleComponents::GenerateTorque_b()
{
  //No attitude control component
  Vector<3> torque_b_(0.0);
  return torque_b_;
};

void SampleComponents::CompoLogSetUp(Logger & logger)
{
  logger.AddLoggable(gyro_);
}
