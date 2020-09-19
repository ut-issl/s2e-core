#include "SampleComponents.h"
#include "Sample_PortConfig.h"

//TODO: Create a base class?
SampleComponents::SampleComponents(const Dynamics* dynamics, const SimulationConfig* config, ClockGenerator* clock_gen, const int sat_id)
  :dynamics_(dynamics), config_(config)
{
  IniAccess iniAccess = IniAccess(config_->sat_file_[sat_id]);

  obc_ = new OBC(clock_gen);
  string gyro_ini_path = iniAccess.ReadString("COMPONENTS_FILE", "gyro_file");
  config_->main_logger_->CopyFileToLogDir(gyro_ini_path);
  gyro_ = new Gyro(InitGyro(clock_gen, 1, GYRO, gyro_ini_path, dynamics_));
}

SampleComponents::~SampleComponents()
{
  delete obc_;
  delete gyro_;
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
