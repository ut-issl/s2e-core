#ifndef __attitude_H__
#define __attitude_H__

#include <string>
using namespace std;

#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Simulation/MCSim/SimulationObject.h"
using libra::Matrix;
using libra::Vector;
using libra::Quaternion;

#include "../../Interface/LogOutput/ILoggable.h"

enum AttCtrlMode
{
  INERTIAL_STABILIZE,
  SUN_POINTING,
  EARTH_CENTER_POINTING,
  VELOCITY_DIRECTION_POINTING,
  ORBIT_NORMAL_POINTING,
  NO_CTRL,
};

class Attitude : public ILoggable
{
public:
  Attitude()
  {
    prop_time_ = 0.0;
    prop_step_ = 0.0;
    omega_b_ = Vector<3>(0.0);
    quaternion_i2b_ = Quaternion(0.0,0.0,0.0,1.0);
    torque_b_ = Vector<3>(0.0);
    h_sc_b_ = Vector<3>(0.0);
    h_rw_b_ = Vector<3>(0.0);
    h_total_b_ = Vector<3>(0.0);
    h_total_i_ = Vector<3>(0.0);
    h_total_ = 0.0;
    inertia_tensor_[0][0] = 1.0;  inertia_tensor_[0][1] = 0.0;  inertia_tensor_[0][2] = 0.0;
    inertia_tensor_[1][0] = 0.0;  inertia_tensor_[1][1] = 1.0;  inertia_tensor_[1][2] = 0.0;
    inertia_tensor_[2][0] = 0.0;  inertia_tensor_[2][1] = 0.0;  inertia_tensor_[2][2] = 1.0;
    inv_inertia_tensor_ = invert(inertia_tensor_);
  }
  virtual ~Attitude(){}

  //Output from this class
  inline double GetPropStep() const { return prop_step_; }
  inline double GetPropTime() const { return prop_time_; }
  inline double GetEnergy() const { return 0.5f*inner_product(omega_b_, inertia_tensor_*omega_b_); }
  inline double GetTotalAngMomNorm() const { return norm(h_total_b_); }
  inline Matrix<3, 3> GetInertiaTensor() const{ return inertia_tensor_;}
  inline Matrix<3, 3> GetInvInertiaTensor() const{ return inv_inertia_tensor_;}
  inline Vector<3> GetOmega_b() const{ return omega_b_;}
  inline Quaternion GetQuaternion_i2b() const{ return quaternion_i2b_;}
  inline Matrix<3, 3> GetDCM_b2i() const{ return quaternion_i2b_.toDCM();}
  inline Matrix<3, 3> GetDCM_i2b() const
  {
    Matrix<3, 3> DCM_b2i = quaternion_i2b_.toDCM();
    return transpose(DCM_b2i);
  }

  //Input to this class
  inline void SetTime(double set){prop_time_ = set;}
  inline void SetPropStep(double set){prop_step_ = set;}
  inline void SetOmega_b(Vector<3> set){omega_b_ = set;}
  inline void SetQuaternion_i2b(Quaternion set){quaternion_i2b_ = set;}
  inline void AddQuaternionOffset(Quaternion offset){quaternion_i2b_ = quaternion_i2b_ * offset;}
  inline void SetTorque_b(Vector<3> set){ torque_b_ = set; }
  inline void SetAngMom_rw(Vector<3> set){h_rw_b_ = set;}
  inline void SetInertiaTensor(const Matrix<3, 3>& set)
  {
    inertia_tensor_ = set;
    inv_inertia_tensor_ = invert(inertia_tensor_);
  }
  inline void AddTorque_b(Vector<3> set){torque_b_ += set;}

  // Setter for Controlled Attitude
  inline void SetMainMode(const AttCtrlMode main_mode){main_mode_ = main_mode;}
  inline void SetSubMode(const AttCtrlMode sub_mode){sub_mode_ = sub_mode;}
  inline void SetQuaternionI2T(const Quaternion quaternion_i2t){quaternion_i2t_ = quaternion_i2t;}
  inline void SetPointingTb(Vector<3> pointing_t_b){ pointing_t_b_ = pointing_t_b;}
  inline void SetPointingSubTb(Vector<3> pointing_sub_t_b){ pointing_sub_t_b_ = pointing_sub_t_b;}

  virtual void Propagate(double endtime) = 0;		// プロパゲーション

  virtual string GetLogHeader() const = 0;
  virtual string GetLogValue() const = 0;

  bool IsCalcEnabled = true;

protected:
  //for RK4
  double prop_time_;	//現在時刻
  double prop_step_;	//積分タイムステップ
  Vector<3> omega_b_;
  Quaternion quaternion_i2b_;
  Vector<3> torque_b_;
  Vector<3> h_sc_b_;
  Vector<3> h_rw_b_;
  Vector<3> h_total_b_;
  Vector<3> h_total_i_;
  double h_total_;
  Matrix<3, 3> inertia_tensor_;
  Matrix<3, 3> inv_inertia_tensor_; // Iner_の逆行列

  //for Controlled Attitude
  AttCtrlMode main_mode_;
  AttCtrlMode sub_mode_;     //for control around pointing direction
  Quaternion quaternion_i2t_;   //ECI->Target
  Vector<3> pointing_t_b_;      //Pointing target on body frame
  Vector<3> pointing_sub_t_b_;  //Pointing sub target on body frame
};
#endif //__attitude_H__
