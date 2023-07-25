/**
 * @file flexible_structure.hpp
 * @brief Component emulation of flexible structure
 */

#ifndef S2E_COMPONENTS_REAL_STRUCTURE_FLEXIBLE_STRUCTURE_HPP_
#define S2E_COMPONENTS_REAL_STRUCTURE_FLEXIBLE_STRUCTURE_HPP_

#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <library/numerical_integration/numerical_integrator.hpp>
#include <library/numerical_integration/runge_kutta_4.hpp>
#include <simulation/spacecraft/structure/structure.hpp>
#include <dynamics/dynamics.hpp>
#include <simulation/spacecraft/installed_components.hpp>

#include "../../base/component.hpp"


#define MODE_NUM 8


/**
 * @class FlexibleStructure
 * @brief Class to show an example to change satellite structure information
 */
class FlexibleStructure : public Component, public ILoggable {
 public:

  struct Mode {
    unsigned id;
    libra::Vector<3> translation_participation_sqrt_kg;
    libra::Vector<3> rotation_participation_sqrt_kg_rad_m;
    double characteristic_angular_frequency_rad_s;
    double damping_ratio;
  };

  // M d2q/dt2 + C dq/dt + K q = f
  class ModeOde : public libra::numerical_integration::InterfaceOde<2 * MODE_NUM> {
  public:
    libra::Vector<2 * MODE_NUM> DerivativeFunction(const double independent_variable, const libra::Vector<2*MODE_NUM>& state) const override;

    void SetCoeff(const libra::Matrix<MODE_NUM, MODE_NUM>& M, const libra::Vector<MODE_NUM>& c, const libra::Vector<MODE_NUM>& k, const libra::Vector<MODE_NUM>& f);

  private:
    libra::Matrix<MODE_NUM, MODE_NUM> M_Lu_;
    size_t index_[MODE_NUM];
    libra::Matrix<MODE_NUM, MODE_NUM> C_;
    libra::Matrix<MODE_NUM, MODE_NUM> K_;
    libra::Vector<MODE_NUM> f_;
  };

  /**
   * @fn FlexibleStructure
   * @brief Constructor with power port
   * @param [in] clock_generator: Clock generator
   * @param [in] structure: Structure information
   */
  FlexibleStructure(const unsigned int prescaler, ClockGenerator* clock_generator, const Structure* structure, const Dynamics* dynamics,
                    InstalledComponents* components, double compo_step_s, double propagation_step_s);

  /**
   * @fn ~FlexibleStructure
   * @brief Destructor
   */
  ~FlexibleStructure();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const override;


  // Getter
  /**
   * @fn GetOutputThrust_b_N
   * @brief Return generated force on the body fixed frame [N]
   */
  inline const Vector<3>& GetForceToBody_b_N() const { return force_to_body_b_N_; };
  /**
   * @fn GetOutputTorque_b_Nm
   * @brief Return generated torque on the body fixed frame [Nm]
   */
  inline const Vector<3>& GetTorqueToBody_b_Nm() const { return torque_to_body_b_Nm_; };

  inline const Vector<MODE_NUM>& GetModeDisplacements_m_sqrt_kg() const { return mode_displacements_m_sqrt_kg_; }
  inline const Vector<MODE_NUM>& GetModeVelocity_m_sqrt_kg_s() const { return mode_velocities_m_sqrt_kg_s_; };
  inline const Vector<MODE_NUM>& GetModeAcceleration_m_sqrt_kg_s2() const { return mode_accelerations_m_sqrt_kg_s2_; };

  double GetEnergy() const;

 protected:
  const Structure* structure_;  //!< Structure information
  const Dynamics* dynamics_;
  InstalledComponents* components_;

  ModeOde ode_;
  libra::numerical_integration::NumericalIntegrator<2 * MODE_NUM>* integrator_;

  Mode modes_[MODE_NUM];
  libra::Vector<3> origin_b_m_{0.0};
  const double propagation_step_s_; //!< RK step time [sec]
  const double compo_step_s_;

  double propagation_time_s_; 
  double compo_time_s_; 

  // outputs
  libra::Vector<3> force_to_body_b_N_{0.0};   //!< Generated thrust on the body fixed frame [N]
  libra::Vector<3> torque_to_body_b_Nm_{0.0};  //!< Generated torque on the body fixed frame [Nm]
  libra::Vector<MODE_NUM> mode_displacements_m_sqrt_kg_;
  libra::Vector<MODE_NUM> mode_velocities_m_sqrt_kg_s_;
  libra::Vector<MODE_NUM> mode_accelerations_m_sqrt_kg_s2_;
};

#endif  // S2E_COMPONENTS_REAL_STRUCTURE_FLEXIBLE_STRUCTURE_HPP_
