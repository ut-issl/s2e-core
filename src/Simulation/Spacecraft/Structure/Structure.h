#pragma once
#include "KinematicsParams.h"
#include "Surface.h"
#include "RMMParams.h"
#include <Simulation/SimulationConfig.h>
#include <vector>
using std::vector;

class Structure
{
public:
  Structure(SimulationConfig* sim_config, const int sat_id);
  ~Structure();
  void Initialize(SimulationConfig* sim_config, const int sat_id);

  //Getter
  inline const vector<Surface>& GetSurfaces() const { return surfaces_; }
  inline const KinematicsParams& GetKinematicsParams() const { return *kinnematics_params_; }
  inline const RMMParams& GetRMMParams() const { return *rmm_params_; }

private:
  KinematicsParams* kinnematics_params_;
  vector<Surface> surfaces_;
  RMMParams* rmm_params_;
};
