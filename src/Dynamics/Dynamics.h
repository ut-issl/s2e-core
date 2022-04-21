#ifndef __dynamics_H__
#define __dynamics_H__

#include <string>

#include "../Library/math/Vector.hpp"
using libra::Vector;

#include "../Environment/Global/SimTime.h"
#include "../Environment/Local/LocalCelestialInformation.h"
#include "../Interface/InitInput/Initialize.h"
#include "../Simulation/SimulationConfig.h"
#include "../Simulation/Spacecraft/Structure/Structure.h"
#include "./Attitude/Attitude.h"
#include "./Orbit/Orbit.h"
#include "./Thermal/Temperature.h"

class RelativeInformation;

class Dynamics {
 public:
  Dynamics(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info, const int sat_id,
           Structure* structure, RelativeInformation* rel_info = (RelativeInformation*)nullptr);
  virtual ~Dynamics();

  void Initialize(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info, const int sat_id,
                  Structure* structure, RelativeInformation* rel_info = (RelativeInformation*)nullptr);
  void Update(const SimTime* sim_time, const LocalCelestialInformation* local_celes_info);
  void LogSetup(Logger& logger);

  // Set
  void AddTorque_b(Vector<3> torque_b);
  void AddForce_b(Vector<3> force_b);
  void AddAcceleration_i(Vector<3> acceleration_i);
  void ClearForceTorque(void);

  // Get
  inline const Attitude& GetAttitude() const { return *attitude_; }
  inline const Orbit& GetOrbit() const { return *orbit_; }
  inline const Temperature& GetTemperature() const { return *temperature_; }
  // Setter
  inline Attitude& SetAttitude() const { return *attitude_; }

  //必要性に疑問を感じる物たち
  double mass_;  // これはここは不適切なきがする．設定ファイルも
  // 慣性系における位置を取得する関数
  // オーバーライドしてセンサーによる推定値を返すのも良い
  virtual Vector<3> GetPosition_i() const;
  // 宇宙機の姿勢を示すクォータニオンを取得する関数
  // オーバーライドしてセンサーによる推定値を返すのも良い
  virtual Quaternion GetQuaternion_i2b() const;

 private:
  Attitude* attitude_;
  Orbit* orbit_;
  Temperature* temperature_;
};

#endif  //__dynamics_H__
