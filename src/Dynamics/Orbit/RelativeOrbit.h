/**
 * @file RelativeOrbit.h
 * @brief Class to propagate relative orbit
 */
#pragma once
#include <Library/RelativeOrbit/RelativeOrbitModels.h>
#include <RelativeInformation/RelativeInformation.h>

#include <Library/math/ODE.hpp>
#include <string>

#include "Orbit.h"

/**
 * @class RelativeOrbit
 * @brief Class to propagate relative orbit
 */
class RelativeOrbit : public Orbit, public libra::ODE<6> {
 public:
  /**
   * @enum RelativeOrbitUpdateMethod
   * @brief Relative orbit update method
   */
  typedef enum { RK4 = 0, STM = 1 } RelativeOrbitUpdateMethod;

  /**
   * @fn RelativeOrbit
   * @brief Constructor
   * @param [in] celes_info: Celestial information
   * @param [in] timestep: Time step [sec]
   * @param [in] reference_sat_id: Reference satellite ID
   * @param [in] initial_relative_position_lvlh: Initial value of relative position at the LVLH frame of reference satellite
   * @param [in] initial_relative_velocity_lvlh: Initial value of relative velocity at the LVLH frame of reference satellite
   * @param [in] update_method: Update method
   * @param [in] relative_dynamics_model_type: Relative dynamics model type
   * @param [in] stm_model_type: State transition matrix type
   * @param [in] rel_info: Relative information
   */
  RelativeOrbit(const CelestialInformation* celes_info, double mu, double timestep, int reference_sat_id, Vector<3> initial_relative_position_lvlh,
                Vector<3> initial_relative_velocity_lvlh, RelativeOrbitUpdateMethod update_method, RelativeOrbitModel relative_dynamics_model_type,
                STMModel stm_model_type, RelativeInformation* rel_info);
  /**
   * @fn ~RelativeOrbit
   * @brief Destructor
   */
  ~RelativeOrbit();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] endtime: End time of simulation [sec]
   * @param [in] current_jd: Current Julian day [day]
   */
  virtual void Propagate(double endtime, double current_jd);

  // Override ODE
  /**
   * @fn RHS
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void RHS(double t, const Vector<6>& state, Vector<6>& rhs);

 private:
  double mu_;             //!< Gravity constant of the center body [m3/s2]
  int reference_sat_id_;  //!< Reference satellite ID
  double prop_time_;      //!< Simulation current time for numerical integration by RK4 [sec]
  double prop_step_;      //!< Step width for RK4 [sec]

  Matrix<6, 6> system_matrix_;  //!< System matrix
  Matrix<6, 6> stm_;            //!< State transition matrix

  Vector<6> initial_state_;           //!< Initial state (Position and Velocity)
  Vector<3> relative_position_lvlh_;  //!< Relative position in the LVLH frame
  Vector<3> relative_velocity_lvlh_;  //!< Relative velocity in the LVLH frame

  RelativeOrbitUpdateMethod update_method_;          //!< Update method
  RelativeOrbitModel relative_dynamics_model_type_;  //!< Relative dynamics model type
  STMModel stm_model_type_;                          //!< State Transition Matrix model type
  RelativeInformation* rel_info_;                    //!< Relative information

  /**
   * @fn InitializeState
   * @brief Initialize state variables
   * @param [in] initial_relative_position_lvlh: Initial value of relative position at the LVLH frame of reference satellite
   * @param [in] initial_relative_velocity_lvlh: Initial value of relative velocity at the LVLH frame of reference satellite
   * @param [in] mu: Gravity constant of the center body [m3/s2]
   * @param [in] init_time: Initialize time [sec]
   */
  void InitializeState(Vector<3> initial_relative_position_lvlh, Vector<3> initial_relative_velocity_lvlh, double mu, double init_time = 0);
  /**
   * @fn CalculateSystemMatrix
   * @brief Calculate system matrix
   * @param [in] relative_dynamics_model_type: Relative dynamics model type
   * @param [in] reference_sat_orbit: Orbit information of reference satellite
   * @param [in] mu: Gravity constant of the center body [m3/s2]
   */
  void CalculateSystemMatrix(RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit, double mu);
  /**
   * @fn CalculateSTM
   * @brief Calculate State Transition Matrix
   * @param [in] stm_model_type: STM model type
   * @param [in] reference_sat_orbit: Orbit information of reference satellite
   * @param [in] mu: Gravity constant of the center body [m3/s2]
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void CalculateSTM(STMModel stm_model_type, const Orbit* reference_sat_orbit, double mu, double elapsed_sec);
  /**
   * @fn PropagateRK4
   * @brief Propagate relative orbit with RK4
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void PropagateRK4(double elapsed_sec);
  /**
   * @fn PropagateSTM
   * @brief Propagate relative orbit with STM
   * @param [in] elapsed_sec: Elapsed time [sec]
   */
  void PropagateSTM(double elapsed_sec);
};
