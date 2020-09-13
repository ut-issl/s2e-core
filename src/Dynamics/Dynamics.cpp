#include "Dynamics.h"
using namespace std;

Dynamics::Dynamics(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info)
{
  Initialize(sim_config, sim_time, local_celes_info);
}

Dynamics::~Dynamics()
{
  delete attitude_;
  delete orbit_;
  delete temperature_;
}

void Dynamics::Initialize(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info)
{
  //Read file name
  IniAccess mainIni = IniAccess(sim_config->ini_base_fname_);
  string orbit_ini_path = mainIni.ReadString("SIM_SETTING", "orbit_file");
  string attitude_ini_path = sim_config->ini_base_fname_;
  // Save ini file
  //config_.logger->CopyFileToLogDir(orbit_ini_path);
  // Initialize
  string center_body_name = local_celes_info->GetGlobalInfo().GetCenterBodyName();
  orbit_ = InitOrbit(orbit_ini_path, sim_time->GetOrbitStepSec(), sim_time->GetCurrentJd(), local_celes_info->GetGlobalInfo().GetGravityConstant(center_body_name.c_str()), "ORBIT");
  attitude_ = InitAttitude(attitude_ini_path, orbit_, local_celes_info);
  temperature_ = InitTemperature(mainIni);
  mass = mainIni.ReadDouble("ATTITUDE", "mass");

  // To get initial value
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());
}

void Dynamics::Update(const SimTime* sim_time, const LocalCelestialInformation* local_celes_info)
{
  //Attitude propagation
  attitude_->Propagate(sim_time->GetElapsedSec());
  //Orbit Propagation
  if (sim_time->GetOrbitPropagateFlag())
  {
    orbit_->Propagate(sim_time->GetCurrentJd());
  }
  //Attitude dependent update
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());

  //Thermal
  std::string sun_str = "SUN";
  char* c_sun = new char[sun_str.size() + 1];
  std::char_traits<char>::copy(c_sun, sun_str.c_str(), sun_str.size() + 1); // string -> char*
  temperature_->Propagate(local_celes_info->GetPosFromSC_b(c_sun), sim_time->GetElapsedSec());
  delete[] c_sun;
}

void Dynamics::ClearForceTorque(void)
{
  Vector<3> zero(0.0);
  attitude_->SetTorque_b(zero);
  orbit_->SetAcceleration_i(zero);
}


void Dynamics::LogSetup(Logger& logger)
{
  logger.AddLoggable(attitude_);
  logger.AddLoggable(orbit_);
}

void Dynamics::AddTorque_b(Vector<3> torque_b)
{
  attitude_->AddTorque_b(torque_b);
}

void Dynamics::AddForce_b(Vector<3> force_b)
{
  orbit_->AddForce_b(force_b, attitude_->GetQuaternion_i2b(), mass);
}

void Dynamics::AddAcceleration_i(Vector<3> acceleration_i)
{
  orbit_->AddAcceleration_i(acceleration_i);
}

Vector<3> Dynamics::GetPosition_i() const
{
  return orbit_->GetSatPosition_i();
}

Quaternion Dynamics::GetQuaternion_i2b() const
{
  return attitude_->GetQuaternion_i2b();
}