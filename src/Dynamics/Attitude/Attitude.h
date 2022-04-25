#ifndef __attitude_H__
#define __attitude_H__

#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/MCSim/SimulationObject.h>

#include <Library/math/MatVec.hpp>
#include <Library/math/Quaternion.hpp>
#include <string>

class Attitude : public ILoggable, public SimulationObject {
 public:
  Attitude(const std::string& sim_object_name = "Attitude");
  virtual ~Attitude() {}

  // Getter
  inline double GetPropStep() const { return prop_step_s_; }
  inline libra::Vector<3> GetOmega_b() const { return omega_b_rad_s_; }
  inline libra::Quaternion GetQuaternion_i2b() const { return quaternion_i2b_; }
  inline libra::Matrix<3, 3> GetDCM_b2i() const { return quaternion_i2b_.toDCM(); }
  inline libra::Matrix<3, 3> GetDCM_i2b() const {
    libra::Matrix<3, 3> DCM_b2i = quaternion_i2b_.toDCM();
    return transpose(DCM_b2i);
  }
  inline double GetTotalAngMomNorm() const { return libra::norm(h_total_b_Nms_); }
  inline double GetEnergy() const { return 0.5f * libra::inner_product(omega_b_rad_s_, inertia_tensor_kgm2_ * omega_b_rad_s_); }
  inline libra::Matrix<3, 3> GetInertiaTensor() const { return inertia_tensor_kgm2_; }
  inline libra::Matrix<3, 3> GetInvInertiaTensor() const { return inv_inertia_tensor_; }

  // Setter
  inline void SetPropStep(double set) { prop_step_s_ = set; }
  inline void SetOmega_b(const libra::Vector<3> set) { omega_b_rad_s_ = set; }
  inline void SetQuaternion_i2b(const libra::Quaternion set) { quaternion_i2b_ = set; }
  inline void AddQuaternionOffset(const libra::Quaternion offset) { quaternion_i2b_ = quaternion_i2b_ * offset; }
  inline void SetTorque_b(const libra::Vector<3> set) { torque_b_Nm_ = set; }
  inline void AddTorque_b(const libra::Vector<3> set) { torque_b_Nm_ += set; }
  inline void SetAngMom_rw(const libra::Vector<3> set) { h_rw_b_Nms_ = set; }
  inline void SetInertiaTensor(const Matrix<3, 3>& set) {
    inertia_tensor_kgm2_ = set;
    inv_inertia_tensor_ = libra::invert(inertia_tensor_kgm2_);
  }

  // base function for derived class
  virtual void Propagate(const double endtime_s) = 0;

  // Loggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // SimulationObject for McSim
  virtual void SetParameters(const MCSimExecutor& mc_sim);

 protected:
  bool is_calc_enabled_ = true;
  double prop_step_s_;  // TODO: consider is it really need here
  libra::Vector<3> omega_b_rad_s_;
  libra::Quaternion quaternion_i2b_;
  libra::Vector<3> torque_b_Nm_;
  libra::Matrix<3, 3> inertia_tensor_kgm2_;
  libra::Matrix<3, 3> inv_inertia_tensor_;  // inverse matrix of Iner_
  libra::Vector<3> h_sc_b_Nms_;
  libra::Vector<3> h_rw_b_Nms_;
  libra::Vector<3> h_total_b_Nms_;
  libra::Vector<3> h_total_i_Nms_;
  double h_total_Nms_;
  double k_sc_J_;  // Rotational Kinetic Energy of Spacecraft [J]

  void CalcAngMom(void);
  void CalcSatRotationalKineticEnergy(void);
};

#endif  //__attitude_H__
