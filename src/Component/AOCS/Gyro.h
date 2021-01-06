#ifndef Gyro_H_
#define Gyro_H_

#include "../../Library/math/Quaternion.hpp"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "../Abstract/SensorBase.h"
#include "../../Dynamics/Dynamics.h"

const size_t kGyroDim=3;

class Gyro : public ComponentBase, public SensorBase<kGyroDim>, public ILoggable
{
public:
  //! コンストラクタ
  /*!
  \q_b2c : 機体座標系(B)→センサ座標系(C)変換Quaternion
  \port_id : OBCとの通信用ポートID
  \MisAlign : センサミスアライメント(3×3行列)
  \current 消費電流値
  \dynamics measureで参照するためのdynamics
  */
  Gyro(
    int prescaler, 
    ClockGenerator* clock_gen,
    const int sensor_id,
    const libra::Quaternion& q_b2c,
    SensorBase& sensor_base,
    const Dynamics *dynamics
  );								
  Gyro(
    int prescaler, 
    ClockGenerator* clock_gen,
    PowerPort* power_port,
    const int sensor_id,
    const libra::Quaternion& q_b2c,
    SensorBase& sensor_base,
    const Dynamics *dynamics
  );						
  ~Gyro();
  // ComponentBase
  void MainRoutine(int count) override;
  // ILoggable
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;
  //Getter
  inline const libra::Vector<kGyroDim>& GetOmegaC(void)const{return omega_c_;}

private:
  libra::Vector<kGyroDim> omega_c_{0.0};
  libra::Quaternion q_b2c_;//! Quaternion from body frame to component frame
  const int sensor_id_;

  const Dynamics* dynamics_;
};

#endif 