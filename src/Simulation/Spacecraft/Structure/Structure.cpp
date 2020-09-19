#include "Structure.h"
#include"../../../Interface/InitInput/Initialize.h"

Structure::Structure(SimulationConfig* sim_config, const int sat_id)
{
  Initialize(sim_config, sat_id);
}

Structure::~Structure()
{
  delete kinnematics_params_;
  delete rmm_params_;
}

void Structure::Initialize(SimulationConfig* sim_config, const int sat_id)
{
  kinnematics_params_ = new KinematicsParams(InitKinematicsParams(sim_config->sat_file_[sat_id]));
  surfaces_ = InitSurfaces(sim_config->sat_file_[sat_id]);
  rmm_params_ = new RMMParams(InitRMMParams(sim_config->sat_file_[sat_id]));
}