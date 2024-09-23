/**
 * @file relative_orbit.hpp
 * @brief Class to propagate relative orbit
 */

#ifndef S2E_DYNAMICS_ORBIT_RELATIVE_ORBIT_HPP_
#define S2E_DYNAMICS_ORBIT_RELATIVE_ORBIT_HPP_

#include <math_physics/math/ordinary_differential_equation.hpp>
#include <math_physics/orbit/relative_orbit_models.hpp>
#include <math_physics/orbit/relative_orbit_yamanaka_ankersen.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>
#include <string>

#include "orbit.hpp"

/**
 * @class RelativeOrbit
 * @brief Class to propagate relative orbit
 */
class RelativeOrbit : public Orbit, public math::OrdinaryDifferentialEquation<6> {
 public:
  /**
   * @enum RelativeOrbitUpdateMethod
   * @brief Relative orbit update method
   */
  typedef enum { kRk4 = 0, kStm = 1 } RelativeOrbitUpdateMethod;

  /**
   * @fn RelativeOrbit
   * @brief Constructor
   * @param [in] celestial_information: Celestial information
   * @param [in] time_step_s: Time step [sec]
   * @param [in] gravity_constant_m3_s2: Gravity constant [m3/s2]
   * @param [in] reference_spacecraft_id: Reference satellite ID
   * @param [in] relative_position_lvlh_m: Initial value of relative position at the LVLH frame of reference satellite
   * @param [in] relative_velocity_lvlh_m_s: Initial value of relative velocity at the LVLH frame of reference satellite
   * @param [in] update_method: Update method
   * @param [in] relative_dynamics_model_type: Relative dynamics model type
   * @param [in] stm_model_type: State transition matrix type
   * @param [in] relative_information: Relative information
   */
  RelativeOrbit(const CelestialInformation* celestial_information, double gravity_constant_m3_s2, double time_step_s, int reference_spacecraft_id,
                math::Vector<3> relative_position_lvlh_m, math::Vector<3> relative_velocity_lvlh_m_s, RelativeOrbitUpdateMethod update_method,
                orbit::RelativeOrbitModel relative_dynamics_model_type, orbit::StmModel stm_model_type, RelativeInformation* relative_information);
  /**
   * @fn ~RelativeOrbit
   * @brief Destructor
   */
  ~RelativeOrbit();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

  // Override OrdinaryDifferentialEquation
  /**
   * @fn DerivativeFunction
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void DerivativeFunction(double t, const Vector<6>& state, Vector<6>& rhs);

 private:
  double gravity_constant_m3_s2_;         //!< Gravity constant of the center body [m3/s2]
  unsigned int reference_spacecraft_id_;  //!< Reference satellite ID
  double propagation_time_s_;             //!< Simulation current time for numerical integration by RK4 [sec]
  double propagation_step_s_;             //!< Step width for RK4 [sec]

  math::Matrix<6, 6> system_matrix_;      //!< System matrix
  math::Matrix<6, 6> stm_;                //!< State transition matrix
  math::Vector<6> correction_term_{0.0};  //!< Correction term for STM calculation

  math::Vector<6> initial_state_;               //!< Initial state (Position and Velocity)
  math::Vector<3> relative_position_lvlh_m_;    //!< Relative position in the LVLH frame
  math::Vector<3> relative_velocity_lvlh_m_s_;  //!< Relative velocity in the LVLH frame

  RelativeOrbitUpdateMethod update_method_;                                 //!< Update method
  orbit::RelativeOrbitModel relative_dynamics_model_type_;                  //!< Relative dynamics model type
  orbit::StmModel stm_model_type_;                                          //!< State Transition Matrix model type
  RelativeInformation* relative_information_;                               //!< Relative information
  orbit::RelativeOrbitYamanakaAnkersen* relative_orbit_yamanaka_ankersen_;  //!< Relative Orbit Calcilater with Yamanaka-Ankersen's STM

  /**
   * @fn InitializeState
   * @brief Initialize state variables
   * @param [in] relative_position_lvlh_m: Initial value of relative position at the LVLH frame of reference satellite
   * @param [in] relative_velocity_lvlh_m_s: Initial value of relative velocity at the LVLH frame of reference satellite
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] initial_time_s: Initialize time [sec]
   */
  void InitializeState(math::Vector<3> relative_position_lvlh_m, math::Vector<3> relative_velocity_lvlh_m_s, double gravity_constant_m3_s2,
                       double initial_time_s = 0);
  /**
   * @fn CalculateSystemMatrix
   * @brief Calculate system matrix
   * @param [in] relative_dynamics_model_type: Relative dynamics model type
   * @param [in] reference_sat_orbit: Orbit information of reference satellite
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   */
  void CalculateSystemMatrix(orbit::RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2);
  /**
   * @fn InitializeStmMatrix
   * @brief Calculate State Transition Matrix
   * @param [in] stm_model_type: STM model type
   * @param [in] reference_sat_orbit: Orbit information of reference satellite
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void InitializeStmMatrix(orbit::StmModel stm_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2, double elapsed_sec);
  /**
   * @fn CalculateStm
   * @brief Calculate State Transition Matrix
   * @param [in] stm_model_type: STM model type
   * @param [in] reference_sat_orbit: Orbit information of reference satellite
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void CalculateStm(orbit::StmModel stm_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2, double elapsed_sec);
  /**
   * @fn PropagateRk4
   * @brief Propagate relative orbit with RK4
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void PropagateRk4(double elapsed_sec);
  /**
   * @fn PropagateStm
   * @brief Propagate relative orbit with STM
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void PropagateStm(double elapsed_sec);
};

#endif  // S2E_DYNAMICS_ORBIT_RELATIVE_ORBIT_HPP_
