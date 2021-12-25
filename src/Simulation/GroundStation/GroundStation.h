#pragma once


#include "../../Interface/InitInput/Initialize.h"
#include "../SimulationConfig.h"
#include "../../Library/math/Vector.hpp"

//↓TODO: 地上局位置を求めるため，Dynamics/Orbit/EarthCenterOrbit.cppのTransECIToGeo()を参考に直接SGPをいじっているが，これをDynamics外に出すissueがあるので，いずれそれと関連して間接的にいじるように変える必要がある
// https://gitlab.com/ut_issl/s2e/s2e_core_oss/-/issues/4
#include "../../Library/sgp4/sgp4unit.h"
#include "../../Library/sgp4/sgp4io.h"
#include "../../Library/sgp4/sgp4ext.h"
#include <math.h>
using namespace std;
#define DEG2RAD 0.017453292519943295769  // PI/180
static gravconsttype whichconst_gs;
//↑



class GroundStation
{
public:
  int gs_id_;  //GroundStationのID
  double latitude_;  //[deg]
  double longitude_;  //[deg]
  double height_;  //[m]
  double elevation_angle_;  //[deg]

  GroundStation(SimulationConfig config, int gs_id_);
  virtual ~GroundStation();

  // ちょっとコピー周りの対応がめんどいのでとりあえずコピー禁止しとく
  // GroundStation(const GroundStation &) = delete;
  // GroundStation& operator= (const GroundStation &) = delete;

  virtual void Initialize(int gs_id);

  virtual void LogSetup(Logger& logger);

  virtual void Update();
  
  Vector<3> GetGSPosition_i() const { return gs_position_i_; }

protected:
  const SimulationConfig config_;
  Vector<3> gs_position_i_;  // 慣性系での地上局位置[m]
};
