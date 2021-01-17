#ifndef __SRPEnvironment_h__
#define __SRPEnvironment_h__
#include "../../Library/math/Vector.hpp"
#include "../../Interface/LogOutput/ILoggable.h"
using libra::Vector;

class SRPEnvironment : public ILoggable
{
public:
  bool IsCalcEnabled = true;

  SRPEnvironment();																//デフォルトコンストラクタ
  void UpdateAllStates(Vector<3>& earth_position_b, Vector<3>& sun_position_b);	//状態を更新、引数は地球中心までのベクトルと太陽中心までのベクトル、単位はm
  double CalcTruePressure() const;														//蝕も考慮した太陽輻射圧を取得
  double CalcPowerDensity() const;                  //Get solar power per unit area considering eclipse [W/m^2]
  double GetPressure() const;															//pressure_を取得(デバッグ用)
  double GetSolarConstant() const;                  //Get solar constant value [W/m^2]
  inline Vector<3> GetSunDirectionFromSC_b() const{return d_sc2sun_b_;}
  double GetShadowFunction() const;                 //Get Shadow function
  inline bool GetIsEclipsed() const { return(shadow_function_ >= 1 ? false : true); } //Returns true if the shadow function is less than 1

  virtual string GetLogHeader() const;													//ログofヘッダー
  virtual string GetLogValue() const;														//ログof値

private:
  double pressure_;																//太陽輻射定数、単位はN/m^2
  double astronomical_unit_;														//1天文単位、単位はm
  double c_;																		//光速、単位はm/s
  double solar_constant_;															//太陽定数、単位はW/m^2
  double r_earth_;																//地球半径、単位はm
  double r_sun_;																	//太陽半径、単位はm
  Vector<3> d_sc2sun_b_;  //Direction Vector from SC to Sun in the body frame
  double shadow_function_ = 1.0;                                     //shadow function

  void CalcShadowFunction(double a, double b, double c, double x, double y);
};

#endif /* SRPEnvironment_h */