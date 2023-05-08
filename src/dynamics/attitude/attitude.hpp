/**
 * @file attitude.hpp
 * @brief Base class for attitude of spacecraft
 */

#ifndef S2E_DYNAMICS_ATTITUDE_ATTITUDE_HPP_
#define S2E_DYNAMICS_ATTITUDE_ATTITUDE_HPP_

#include <library/logger/loggable.hpp>
#include <library/math/matrix_vector.hpp>
#include <library/math/quaternion.hpp>
#include <simulation/monte_carlo_simulation/simulation_object.hpp>
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
   * @param [in] simulation_object_name: Simulation object name for Monte-Carlo simulation
   */
  Attitude(const libra::Matrix<3, 3>& inertia_tensor_kgm2, const std::string& simulation_object_name = "attitude");
  /**
   * @fn ~Attitude
   * @brief Destructor
   */
  virtual ~Attitude() {}

  // Getter
  /**
   * @fn GetPropStep_s
   * @brief Return propagation step [sec]
   */
  inline double GetPropStep_s() const { return propagation_step_s_; }
  /**
   * @fn GetAngularVelocity_b_rad_s
   * @brief Return angular velocity of spacecraft body-fixed frame with respect to the inertial frame [rad/s]
   */
  inline libra::Vector<3> GetAngularVelocity_b_rad_s() const { return angular_velocity_b_rad_s_; }
  /**
   * @fn GetQuaternion_i2b
   * @brief Return attitude quaternion from the inertial frame to the body fixed frame
   */
  inline libra::Quaternion GetQuaternion_i2b() const { return quaternion_i2b_; }
  /**
   * @fn GetTotalAngularMomentNorm_Nms
   * @brief Return norm of total angular momentum of the spacecraft [Nms]
   */
  inline double GetTotalAngularMomentNorm_Nms() const { return angular_momentum_total_b_Nms_.CalcNorm(); }
  /**
   * @fn GetKineticEnergy_J
   * @brief Return rotational Kinetic Energy of Spacecraft [J]
   */
  inline double GetKineticEnergy_J() const { return kinetic_energy_J_; }
  /**
   * @fn GetInertiaTensor_b_kgm2
   * @brief Return inertia tensor [kg m^2]
   */
  inline libra::Matrix<3, 3> GetInertiaTensor_b_kgm2() const { return inertia_tensor_kgm2_; }

  // Setter
  /**
   * @fn SetAngularVelocity_b_rad_s
   * @brief Set angular velocity of the body fixed frame with respect to the inertial frame [rad/s]
   */
  inline void SetAngularVelocity_b_rad_s(const libra::Vector<3> angular_velocity_b_rad_s) { angular_velocity_b_rad_s_ = angular_velocity_b_rad_s; }
  /**
   * @fn SetQuaternion_i2b
   * @brief Set attitude quaternion from the inertial frame to the body frame
   */
  inline void SetQuaternion_i2b(const libra::Quaternion quaternion_i2b) { quaternion_i2b_ = quaternion_i2b; }
  /**
   * @fn SetTorque_b_Nm
   * @brief Set torque acting on the spacecraft on the body fixed frame [Nm]
   */
  inline void SetTorque_b_Nm(const libra::Vector<3> torque_b_Nm) { torque_b_Nm_ = torque_b_Nm; }
  /**
   * @fn AddTorque_b_Nm
   * @brief Add torque acting on the spacecraft on the body fixed frame [Nm]
   */
  inline void AddTorque_b_Nm(const libra::Vector<3> torque_b_Nm) { torque_b_Nm_ += torque_b_Nm; }
  /**
   * @fn SetRwAngularMomentum_b_Nms
   * @brief Set angular momentum of reaction wheel in the body fixed frame [Nms]
   */
  inline void SetRwAngularMomentum_b_Nms(const libra::Vector<3> angular_momentum_rw_b_Nms) {
    angular_momentum_reaction_wheel_b_Nms_ = angular_momentum_rw_b_Nms;
  }

  /**
   * @fn Propagate
   * @brief Pure virtual function of attitude propagation
   * @param [in] end_time_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double end_time_s) = 0;

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
  virtual void SetParameters(const MonteCarloSimulationExecutor& mc_simulator);

 protected:
  bool is_calc_enabled_ = true;                //!< Calculation flag
  double propagation_step_s_;                  //!< Propagation step [sec]
  libra::Vector<3> angular_velocity_b_rad_s_;  //!< Angular velocity of spacecraft body fixed frame with respect to the inertial frame [rad/s]
  libra::Quaternion quaternion_i2b_;           //!< Attitude quaternion from the inertial frame to the body fixed frame
  libra::Vector<3> torque_b_Nm_;               //!< Torque in the body fixed frame [Nm]

  libra::Matrix<3, 3> inertia_tensor_kgm2_;    //!< Inertia tensor of the spacecraft [kg m^2] TODO: Move to Structure
  libra::Matrix<3, 3> inv_inertia_tensor_;     //!< Inverse matrix of the inertia tensor

  libra::Vector<3> angular_momentum_spacecraft_b_Nms_;      //!< Angular momentum of spacecraft in the body fixed frame [Nms]
  libra::Vector<3> angular_momentum_reaction_wheel_b_Nms_;  //!< Angular momentum of reaction wheel in the body fixed frame [Nms]
  libra::Vector<3> angular_momentum_total_b_Nms_;           //!< Total angular momentum of spacecraft in the body fixed frame [Nms]
  libra::Vector<3> angular_momentum_total_i_Nms_;           //!< Total angular momentum of spacecraft in the inertial frame [Nms]
  double angular_momentum_total_Nms_;                       //!< Norm of total angular momentum [Nms]
  double kinetic_energy_J_;                                 //!< Rotational Kinetic Energy of Spacecraft [J]

  /**
   * @fn CalcAngularMomentum
   * @brief Calculate angular momentum
   */
  void CalcAngularMomentum(void);
};

#endif  // S2E_DYNAMICS_ATTITUDE_ATTITUDE_HPP_
