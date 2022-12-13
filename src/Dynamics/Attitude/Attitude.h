/**
 * @file Attitude.h
 * @brief Base class for attitude of spacecraft
 */
#ifndef __attitude_H__
#define __attitude_H__

#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/MCSim/SimulationObject.h>

#include <Library/math/MatVec.hpp>
#include <Library/math/Quaternion.hpp>
#include <string>

/**
 * @class Attitude
 * @brief Base class for attitude of spacecraft
 */
class Attitude : public ILoggable, public SimulationObject {
 public:
  /**
   * @fn Attitude
   * @brief Constructor
   * @param [in] sim_object_name: Simulation object name for Monte-Carlo simulation
   */
  Attitude(const std::string& sim_object_name = "Attitude");
  /**
   * @fn ~Attitude
   * @brief Destructor
   */
  virtual ~Attitude() {}

  // Getter
  /**
   * @fn GetPropStep
   * @brief Return propagation step [sec]
   */
  inline double GetPropStep() const { return prop_step_s_; }
  /**
   * @fn GetOmega_b
   * @brief Return angular velocity of spacecraft body-fixed frame with respect to the inertial frame [rad/s]
   */
  inline libra::Vector<3> GetOmega_b() const { return omega_b_rad_s_; }
  /**
   * @fn GetQuaternion_i2b
   * @brief Return attitude quaternion from the inertial frame to the body fixed frame
   */
  inline libra::Quaternion GetQuaternion_i2b() const { return quaternion_i2b_; }
  /**
   * @fn GetDCM_b2i
   * @brief Return attitude direction cosine matrix from the body fixed frame to the inertial frame
   * @note TODO: Check correctness i2b? b2i?
   */
  inline libra::Matrix<3, 3> GetDCM_b2i() const { return quaternion_i2b_.toDCM(); }
  /**
   * @fn GetDCM_i2b
   * @brief Return attitude direction cosine matrix from the inertial frame to the body fixed frame
   * @note TODO: Check correctness i2b? b2i?
   */
  inline libra::Matrix<3, 3> GetDCM_i2b() const {
    libra::Matrix<3, 3> DCM_b2i = quaternion_i2b_.toDCM();
    return transpose(DCM_b2i);
  }
  /**
   * @fn GetTotalAngMomNorm
   * @brief Return norm of total angular momentum of the spacecraft [Nms]
   */
  inline double GetTotalAngMomNorm() const { return libra::norm(h_total_b_Nms_); }
  /**
   * @fn GetEnergy
   * @brief Return rotational Kinetic Energy of Spacecraft [J]
   * @note TODO: Consider to use k_sc_J_
   */
  inline double GetEnergy() const { return 0.5f * libra::inner_product(omega_b_rad_s_, inertia_tensor_kgm2_ * omega_b_rad_s_); }
  /**
   * @fn GetInertiaTensor
   * @brief Return inertia tensor [kg m^2]
   */
  inline libra::Matrix<3, 3> GetInertiaTensor() const { return inertia_tensor_kgm2_; }
  /**
   * @fn GetInvInertiaTensor
   * @brief Return inverse matrix of inertia tensor
   */
  inline libra::Matrix<3, 3> GetInvInertiaTensor() const { return inv_inertia_tensor_; }

  // Setter
  /**
   * @fn SetPropStep
   * @brief Set propagation step [sec]
   */
  inline void SetPropStep(double set) { prop_step_s_ = set; }
  /**
   * @fn SetOmega_b
   * @brief Set angular velocity of the body fixed frame with respect to the inertial frame [rad/s]
   */
  inline void SetOmega_b(const libra::Vector<3> set) { omega_b_rad_s_ = set; }
  /**
   * @fn SetQuaternion_i2b
   * @brief Set attitude quaternion from the inertial frame to the body frame
   */
  inline void SetQuaternion_i2b(const libra::Quaternion set) { quaternion_i2b_ = set; }
  /**
   * @fn AddQuaternionOffset
   * @brief Add quaternion offset rotation
   */
  inline void AddQuaternionOffset(const libra::Quaternion offset) { quaternion_i2b_ = quaternion_i2b_ * offset; }
  /**
   * @fn SetTorque_b
   * @brief Set torque acting on the spacecraft on the body fixed frame [Nm]
   */
  inline void SetTorque_b(const libra::Vector<3> set) { torque_b_Nm_ = set; }
  /**
   * @fn AddTorque_b
   * @brief Add torque acting on the spacecraft on the body fixed frame [Nm]
   */
  inline void AddTorque_b(const libra::Vector<3> set) { torque_b_Nm_ += set; }
  /**
   * @fn SetAngMom_rw
   * @brief Set angular momentum of reaction wheel in the body fixed frame [Nms]
   */
  inline void SetAngMom_rw(const libra::Vector<3> set) { h_rw_b_Nms_ = set; }
  /**
   * @fn SetInertiaTensor
   * @brief Set inertia tensor of the spacecraft [kg m2]
   */
  inline void SetInertiaTensor(const Matrix<3, 3>& set) {
    inertia_tensor_kgm2_ = set;
    inv_inertia_tensor_ = libra::invert(inertia_tensor_kgm2_);
  }

  /**
   * @fn Propagate
   * @brief Pure virtual function of attitude propagation
   * @param [in] endtime_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double endtime_s) = 0;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  // SimulationObject for McSim
  virtual void SetParameters(const MCSimExecutor& mc_sim);

 protected:
  bool is_calc_enabled_ = true;              //!< Calculation flag
  double prop_step_s_;                       //!< Propagation step [sec] TODO: consider is it really need here
  libra::Vector<3> omega_b_rad_s_;           //!< Angular velocity of spacecraft body fixed frame with respect to the inertial frame [rad/s]
  libra::Quaternion quaternion_i2b_;         //!< Attitude quaternion from the inertial frame to the body fixed frame
  libra::Vector<3> torque_b_Nm_;             //!< Torque in the body fixed frame [Nm]
  libra::Matrix<3, 3> inertia_tensor_kgm2_;  //!< Inertia tensor of the spacecraft [kg m^2] TODO: Move to Structure
  libra::Matrix<3, 3> inv_inertia_tensor_;   //!< Inverse matrix of the inertia tensor
  libra::Vector<3> h_sc_b_Nms_;              //!< Angular momentum of spacecraft in the body fixed frame [Nms]
  libra::Vector<3> h_rw_b_Nms_;              //!< Angular momentum of reaction wheel in the body fixed frame [Nms]
  libra::Vector<3> h_total_b_Nms_;           //!< Total angular momentum of spacecraft in the body fixed frame [Nms]
  libra::Vector<3> h_total_i_Nms_;           //!< Total angular momentum of spacecraft in the inertial frame [Nms]
  double h_total_Nms_;                       //!< Norm of total angular momentum [Nms]
  double k_sc_J_;                            //!< Rotational Kinetic Energy of Spacecraft [J]

  /**
   * @fn CalcAngMom
   * @brief Calculate angular momentum
   */
  void CalcAngMom(void);
  /**
   * @fn CalcSatRotationalKineticEnergy
   * @brief Calculate rotational Kinetic Energy of Spacecraft
   */
  void CalcSatRotationalKineticEnergy(void);
};

#endif  //__attitude_H__
