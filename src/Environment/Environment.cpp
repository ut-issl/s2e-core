#include "Environment.h"
#include "Atmosphere.h"
#include "MagEnvironment.h"
#include "SRPEnvironment.h"
#include "CelestialInformation.h"
#include "../Interface/InitInput/Initialize.h"
#include "../Simulation/Spacecraft/Spacecraft.h"
#include "../Dynamics/Attitude/Attitude.h"
#include "../Dynamics/Orbit/Orbit.h"

Envir::Envir(string base_ini_fname)
  :base_ini_fname_(base_ini_fname)
{
  Initialize();
}

Envir::~Envir()
{
  delete magnet;
  delete srp;
  delete atmosphere;
}

void Envir::Initialize()
{
  IniAccess iniAccess = IniAccess(base_ini_fname_);
  ini_fname_ = iniAccess.ReadString("SIM_SETTING", "env_file");

  magnet     = new MagEnvironment(InitMagEnvironment(ini_fname_));
  srp        = new SRPEnvironment(InitSRPEnvironment(ini_fname_));
  atmosphere = new Atmosphere(InitAtmosphere(ini_fname_));
}

void Envir::Update(Spacecraft& spacecraft, SimTime& simTime)
{
  auto& orbit     = spacecraft.dynamics_->GetOrbit();
  auto& celestial = spacecraft.dynamics_->GetCelestial();
  auto& attitude  = spacecraft.dynamics_->GetAttitude();

  magnet->CalcMag(simTime.GetCurrentDecyear(), simTime.GetCurrentSidereal(),
    orbit.GetLatLonAlt(), attitude.GetQuaternion_i2b());
  Vector<3> v1 = celestial.GetPosFromSC_b("EARTH");
  Vector<3> v2 = celestial.GetPosFromSC_b("SUN");
  srp->UpdateAllStates(v1, v2);
  atmosphere->CalcAirDensity(simTime.GetCurrentDecyear(),  simTime.GetEndSec(), orbit.GetLatLonAlt());
}

void Envir::LogSetup(Logger & logger)
{
  logger.AddLoggable(magnet);
  logger.AddLoggable(srp);
  logger.AddLoggable(atmosphere);
  //Log ini file
  logger.CopyFileToLogDir(ini_fname_);
}

