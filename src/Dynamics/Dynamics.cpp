
#include "Dynamics.h"
using namespace std;


Dynamics::Dynamics(SimulationConfig config, string AttitudeName, string OrbitName)
  :config_(config), AttitudeName(AttitudeName), OrbitName(OrbitName)
{
  Initialize();
}

Dynamics::~Dynamics()
{
  delete attitude_;
  delete orbit_;
  delete celestial_;
  delete hip_;
}

void Dynamics::Initialize()
{
  string ini_fname = config_.mainIniPath;
  IniAccess mainIni = IniAccess(ini_fname);
  string orbit_ini_path = mainIni.ReadString("SIM_SETTING", "orbit_file");
  string celestial_ini_path = mainIni.ReadString("SIM_SETTING", "celestial_file");
  string hip_ini_path = mainIni.ReadString("SIM_SETTING", "env_file");

  config_.logger->CopyFileToLogDir(orbit_ini_path);
  config_.logger->CopyFileToLogDir(celestial_ini_path);

  orbit_ = InitOrbit(orbit_ini_path, config_.simTime->GetOrbitStepSec(), config_.simTime->GetCurrentJd(), OrbitName);
  celestial_ = new CelestialInformation(InitCelesInfo(celestial_ini_path));
  attitude_ = InitAttitude(ini_fname, orbit_, celestial_);
  hip_ = new HipparcosCatalogue(InitHipCatalogue(hip_ini_path));

  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());

  celestial_->UpdateAllObjectsInfo(
    config_.simTime->GetCurrentJd(), orbit_->GetSatPosition_i(),
    orbit_->GetSatVelocity_i(), attitude_->GetQuaternion_i2b(), attitude_->GetOmega_b());
  celestial_->CalcAllPosVel_b(attitude_->GetQuaternion_i2b(), attitude_->GetOmega_b());

  temperature_ = InitTemperature(mainIni);

  mass = mainIni.ReadDouble(AttitudeName.c_str(), "mass");
}

void Dynamics::Update()
{
  attitude_->Propagate(config_.simTime->GetElapsedSec());

  //Orbit Propagation, Celestial Information Update
  // 一定間隔で伝搬計算を行う
  if (config_.simTime->GetOrbitPropagateFlag())
  {
    orbit_->Propagate(config_.simTime->GetCurrentJd());
    celestial_->UpdateAllObjectsInfo(config_.simTime->GetCurrentJd(),
      orbit_->GetSatPosition_i(), orbit_->GetSatVelocity_i(), attitude_->GetQuaternion_i2b(), attitude_->GetOmega_b());
  }
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());
  //姿勢によって変化する部分のみ更新
  celestial_->CalcAllPosVel_b(attitude_->GetQuaternion_i2b(), attitude_->GetOmega_b());
  attitude_->SetTorque_b(Vector<3>(0));

  std::string sun_str = "SUN";
  char* c_sun = new char[sun_str.size() + 1];
  std::char_traits<char>::copy(c_sun, sun_str.c_str(), sun_str.size() + 1); // string -> char*
  temperature_->Propagate(celestial_->GetPosFromSC_b(c_sun), config_.simTime->GetElapsedSec());
  delete[] c_sun;
}

void Dynamics::LogSetup(Logger& logger)
{
  logger.AddLoggable(attitude_);
  logger.AddLoggable(orbit_);
  logger.AddLoggable(celestial_);
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

double Dynamics::GetCurrentJd() const
{
  return config_.simTime->GetCurrentJd();
}
