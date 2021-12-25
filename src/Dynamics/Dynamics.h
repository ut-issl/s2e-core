#ifndef __dynamics_H__
#define __dynamics_H__

#include <string>
using namespace std;

#include "../Library/math/Vector.hpp"
using libra::Vector;

#include "./Attitude/Attitude.h"
#include "./Orbit/Orbit.h"
#include "../Environment/CelestialInformation.h"
#include "./Thermal/Temperature.h"
#include "../Environment/HipparcosCatalogue.h"


#include "../Interface/InitInput/Initialize.h"
#include "../Simulation/SimulationConfig.h"


class Dynamics
{
public:
  Dynamics(SimulationConfig config, string AttitudeName, string OrbitName);
  ~Dynamics();
  void Initialize();
  void Update();
  void LogSetup(Logger& logger);

  inline const Attitude& GetAttitude() const { return *attitude_; }
  inline const Orbit& GetOrbit() const { return *orbit_; }
  inline const CelestialInformation& GetCelestial() const { return *celestial_; }
  inline const Temperature& GetTemperature() const { return *temperature_;  }  

  void AddTorque_b(Vector<3> torque_b);
  void AddForce_b(Vector<3> force_b);
  void AddAcceleration_i(Vector<3> acceleration_i);


  // 慣性系における位置を取得する関数
  // オーバーライドしてセンサーによる推定値を返すのも良い
  virtual Vector<3> GetPosition_i() const;

  // 宇宙機の姿勢を示すクォータニオンを取得する関数
  // オーバーライドしてセンサーによる推定値を返すのも良い
  virtual Quaternion GetQuaternion_i2b() const;

  double GetCurrentJd() const;  //TODO: 現状では，地上局位置をECI2ECEF変換するために外部にユリウス時を持ち出す必要がある（今後issue4で整備したい）

  Attitude*             attitude_;
  Orbit*                orbit_;
  CelestialInformation* celestial_;
  Temperature*          temperature_;
  HipparcosCatalogue* hip_;
  double mass;        // これはここは不適切なきがする．設定ファイルも

private:
  const string AttitudeName;
  const string OrbitName;

  const SimulationConfig config_;       // これを色んな所においていいのか？ 問題あり
};


#endif //__dynamics_H__
