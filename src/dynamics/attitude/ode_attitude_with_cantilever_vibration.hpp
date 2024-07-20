/**
 * @file ode_attitude_with_cantilever_vibration.hpp
 * @brief Class to implement Ordinary Differential Equations for Attitude propagator with cantilever vibration
 */

#ifndef S2E_DYNAMICS_ATTITUDE_ODE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_
#define S2E_DYNAMICS_ATTITUDE_ODE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_

#include <math_physics/numerical_integration/interface_ode.hpp>
#include <utilities/macros.hpp>

#include "attitude.hpp"

namespace libra::numerical_integration {
/**
 * @class AttitudeWithCantileverVibrationOde
 * @brief Class to implement Ordinary Differential Equations for Attitude with Cantilever Vibration
 * @note  State variables in this ODE compose the following elenents (in order): angular_velocity_b_rad_s_ (3-dimension),
 *        angular_velocity_cantilever_rad_s_ (3-dimension), quaternion_i2b_ (4-dmension), and euler_angular_cantilever_rad_ (3-dimension)
 */
class AttitudeWithCantileverVibrationOde : public InterfaceOde<13> {
 public:
  /**
   * @fn SetStateFromPhysicalQuantities
   * @brief Set state for calculating the ordinary differential equation from physical quantities
   * @param [in] angular_velocity_b_rad_s: Angular velocity of the spacecraft @ body-fixed frame [rad/s]
   * @param [in] angular_velocity_cantilever_rad_s: Angular velocity of the cantilever @ body frame [rad/s]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   * @param [in] euler_angule_cantilever_rad: Euler angle of the cantilever@ body-fixed frame [rad]
   */
  math::Vector<13> SetStateFromPhysicalQuantities(const math::Vector<3> angular_velocity_b_rad_s,
                                                  const math::Vector<3> angular_velocity_cantilever_rad_s, const math::Quaternion quaternion_i2b,
                                                  const math::Vector<3> euler_angule_cantilever_rad) const {
    math::Vector<13> state;
    for (size_t i = 0; i < 3; i++) {
      state[i] = angular_velocity_b_rad_s[i];
    }
    for (size_t i = 0; i < 3; i++) {
      state[i + 3] = angular_velocity_cantilever_rad_s[i];
    }
    for (size_t i = 0; i < 4; i++) {
      state[i + 6] = quaternion_i2b[i];
    }
    for (size_t i = 0; i < 3; i++) {
      state[i + 10] = euler_angule_cantilever_rad[i];
    }
    return state;
  }

  /**
   * @fn SetPhysicalQuantitiesFromState
   * @brief Set physical quantities from state acquired by calculation of the ordinary differential equation
   * @param [in] state: state variables used to calculate the ordinary differential equation
   */
  void SetPhysicalQuantitiesFromState(const math::Vector<13> state, math::Vector<3>& angular_velocity_b_rad_s,
                                      math::Vector<3>& angular_velocity_cantilever_rad_s, math::Quaternion& quaternion_i2b,
                                      math::Vector<3>& euler_angular_cantilever_rad) const {
    for (size_t i = 0; i < 3; i++) {
      angular_velocity_b_rad_s[i] = state[i];
    }
    for (size_t i = 0; i < 3; i++) {
      angular_velocity_cantilever_rad_s[i] = state[i + 3];
    }
    for (size_t i = 0; i < 4; i++) {
      quaternion_i2b[i] = state[i + 6];
    }
    for (size_t i = 0; i < 3; i++) {
      euler_angular_cantilever_rad[i] = state[i + 10];
    }
  }

  math::Vector<13> DerivativeFunction(const double time_s, const math::Vector<13>& state) const override {
    UNUSED(time_s);

    math::Vector<13> output;

    math::Vector<3> omega_b_rad_s;
    math::Vector<3> omega_cantilever_rad_s;
    math::Quaternion quaternion_i2b;
    math::Vector<3> euler_angle_cantilever_rad;

    SetPhysicalQuantitiesFromState(state, omega_b_rad_s, omega_cantilever_rad_s, quaternion_i2b, euler_angle_cantilever_rad);

    math::Vector<3> angular_momentum_total_b_Nms = (previous_inertia_tensor_kgm2_ * omega_b_rad_s) + angular_momentum_reaction_wheel_b_Nms_;
    math::Vector<3> net_torque_b_Nm = torque_b_Nm_ - math::OuterProduct(omega_b_rad_s, angular_momentum_total_b_Nms) - torque_inertia_tensor_b_Nm_;

    math::Vector<3> angular_accelaration_cantilever_rad_s2 =
        -(inverse_equivalent_inertia_tensor_cantilever_ *
          (attenuation_coefficient_ * omega_cantilever_rad_s + spring_coefficient_ * euler_angle_cantilever_rad)) -
        inverse_inertia_tensor_ * net_torque_b_Nm;

    math::Vector<3> rhs = inverse_inertia_tensor_ * (net_torque_b_Nm - inertia_tensor_cantilever_kgm2_ * angular_accelaration_cantilever_rad_s2);

    for (size_t i = 0; i < 3; ++i) {
      output[i] = rhs[i];
    }

    for (size_t i = 0; i < 3; i++) {
      output[i + 3] = angular_accelaration_cantilever_rad_s2[i];
    }

    math::Vector<4> d_quaternion = 0.5 * CalcAngularVelocityMatrix(omega_b_rad_s) * (math::Vector<4>(quaternion_i2b));

    for (size_t i = 0; i < 4; i++) {
      output[i + 6] = d_quaternion[i];
    }

    for (size_t i = 0; i < 3; i++) {
      output[i + 10] = omega_cantilever_rad_s[i];
    }

    // The function is used to output the derivative of each corresponding physical quantity.
    output = SetStateFromPhysicalQuantities(rhs, angular_accelaration_cantilever_rad_s2, math::Quaternion(d_quaternion), omega_cantilever_rad_s);

    return output;
  }

  // Getter
  /**
   * @fn GetAttenuationCoefficient
   * @brief Get attenuation coefficient
   */
  inline double GetAttenuationCoefficient() { return attenuation_coefficient_; }
  /**
   * @fn GetSpringCoefficient
   * @brief Get spring coefficient
   */
  inline double GetSpringCoefficient() { return spring_coefficient_; }
  /**
   * @fn GetTorqueInertiaTensor_b_Nm
   * @brief Get torque generated by inertia tensor [Nm]
   */
  inline math::Vector<3> GetTorqueInertiaTensor_b_Nm() { return torque_inertia_tensor_b_Nm_; }
  /**
   * @fn GetInverseInertiaTensor
   * @brief Get inverse of inertia tensor
   */
  inline math::Matrix<3, 3> GetInverseInertiaTensor() { return inverse_inertia_tensor_; }
  /**
   * @fn GetPreviousInertiaTensor_kgm2
   * @brief Get previous inertia tensor [kgm2]
   */
  inline math::Matrix<3, 3> GetPreviousInertiaTensor_kgm2() { return previous_inertia_tensor_kgm2_; }
  /**
   * @fn GetInertiaTensorCantilever_kgm2
   * @brief Get inertia tensor of the cantilever [kgm2]
   */
  inline math::Matrix<3, 3> GetInertiaTensorCantilever_kgm2() { return inertia_tensor_cantilever_kgm2_; }
  /**
   * @fn GetInverseEquivalentInertiaTensorCantilever
   * @brief Get inverse of inertia tensor of the cantilever
   */
  inline math::Matrix<3, 3> GetInverseEquivalentInertiaTensorCantilever() { return inverse_equivalent_inertia_tensor_cantilever_; }

  // Setter
  /**
   * @fn SetAttenuationCoefficient
   * @brief Set attenuation coefficient
   */
  inline void SetAttenuationCoefficient(const double attenuation_coefficient) { attenuation_coefficient_ = attenuation_coefficient; }
  /**
   * @fn SetSpringCoefficient
   * @brief Set spring coefficient
   */
  inline void SetSpringCoefficient(const double spring_coefficient) { spring_coefficient_ = spring_coefficient; }
  /**
   * @fn SetTorque_b_Nm
   * @brief Set torque acting on the spacecraft on the body fixed frame [Nm]
   */
  inline void SetTorque_b_Nm(const math::Vector<3> torque_b_Nm) { torque_b_Nm_ = torque_b_Nm; }
  /**
   * @fn SetTorqueInertiaTensor_b_Nm
   * @brief Set torque generated by inertia tensor [Nm]
   */
  inline void SetTorqueInertiaTensor_b_Nm(const math::Vector<3> torque_inertia_tensor_b_Nm) {
    torque_inertia_tensor_b_Nm_ = torque_inertia_tensor_b_Nm;
  }
  /**
   * @fn SetAngularMomentumReactionWheel_b_Nms
   * @brief Set angular momentum of reaction wheel in the body fixed frame [Nms]
   */
  inline void SetAngularMomentumReactionWheel_b_Nms(const math::Vector<3> angular_momentum_reaction_wheel_b_Nms) {
    angular_momentum_reaction_wheel_b_Nms_ = angular_momentum_reaction_wheel_b_Nms;
  }
  /**
   * @fn SetInverseInertiaTensor
   * @brief Set inverse of inertia tensor
   */
  inline void SetInverseInertiaTensor(const math::Matrix<3, 3> inverse_inertia_tensor) { inverse_inertia_tensor_ = inverse_inertia_tensor; }
  /**
   * @fn SetPreviousInertiaTensor_kgm2
   * @brief Set previous inertia tensor [kgm2]
   */
  inline void SetPreviousInertiaTensor_kgm2(const math::Matrix<3, 3> previous_inertia_tensor_kgm2) {
    previous_inertia_tensor_kgm2_ = previous_inertia_tensor_kgm2;
  }
  /**
   * @fn SetInertiaTensorCantilever_kgm2
   * @brief Set inertia tensor of the cantilever [kgm2]
   */
  inline void SetInertiaTensorCantilever_kgm2(const math::Matrix<3, 3> inertia_tensor_cantilever_kgm2) {
    inertia_tensor_cantilever_kgm2_ = inertia_tensor_cantilever_kgm2;
  }
  /**
   * @fn SetInverseEquivalentInertiaTensorCantilever
   * @brief Set inverse of inertia tensor of the cantilever
   */
  inline void SetInverseEquivalentInertiaTensorCantilever(const math::Matrix<3, 3> inverse_equivalent_inertia_tensor_cantilever) {
    inverse_equivalent_inertia_tensor_cantilever_ = inverse_equivalent_inertia_tensor_cantilever;
  }

 protected:
  double attenuation_coefficient_ = 0.0;                                  //!< Attenuation coefficient
  double spring_coefficient_ = 0.0;                                       //!< Spring coefficient
  math::Vector<3> torque_b_Nm_{0.0};                                      //!< Torque in the body fixed frame [Nm]
  math::Vector<3> torque_inertia_tensor_b_Nm_{0.0};                       //!< Torque generated by inertia tensor [Nm]
  math::Vector<3> angular_momentum_reaction_wheel_b_Nms_{0.0};            //!< Angular momentum of reaction wheel in the body fixed frame [Nms]
  math::Matrix<3, 3> inverse_inertia_tensor_{0.0};                        //!< Inverse of inertia tensor
  math::Matrix<3, 3> previous_inertia_tensor_kgm2_{0.0};                  //!< Previous inertia tensor [kgm2]
  math::Matrix<3, 3> inertia_tensor_cantilever_kgm2_{0.0};                //!< Inertia tensor of the cantilever [kgm2]
  math::Matrix<3, 3> inverse_equivalent_inertia_tensor_cantilever_{0.0};  //!< Inverse of inertia tensor of the cantilever
};

}  // namespace libra::numerical_integration

#endif  // S2E_DYNAMICS_ATTITUDE_ODE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_
