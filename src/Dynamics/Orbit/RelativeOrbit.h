#pragma once
#include <Library/RelativeOrbit/RelativeOrbitModels.h>
#include <RelativeInformation/RelativeInformation.h>

#include <Library/math/ODE.hpp>
#include <string>

#include "Orbit.h"

class RelativeOrbit : public Orbit, public libra::ODE<6> {
 public:
  typedef enum { RK4 = 0, STM = 1 } RelativeOrbitUpdateMethod;

  RelativeOrbit(const CelestialInformation* celes_info, double mu, double timestep, int wgs, double current_jd, int reference_sat_id,
                Vector<3> initial_relative_position_lvlh, Vector<3> initial_relative_velocity_lvlh, RelativeOrbitUpdateMethod update_method,
                RelativeOrbitModel relative_dynamics_model_type, STMModel stm_model_type, RelativeInformation* rel_info);
  ~RelativeOrbit();

  virtual void Propagate(double endtime, double current_jd);

  virtual void RHS(double t, const Vector<6>& state, Vector<6>& rhs);

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  double mu_;
  int reference_sat_id_;

  double prop_time_;  // Simulation current time for numerical integration by RK4
  double prop_step_;  // delta t for RK4

  Matrix<6, 6> system_matrix_;
  Matrix<6, 6> stm_;

  Vector<6> initial_state_;
  Vector<3> relative_position_lvlh_;
  Vector<3> relative_velocity_lvlh_;

  RelativeOrbitUpdateMethod update_method_;
  RelativeOrbitModel relative_dynamics_model_type_;
  STMModel stm_model_type_;
  RelativeInformation* rel_info_;

  void InitializeState(Vector<3> initial_relative_position_lvlh, Vector<3> initial_relative_velocity_lvlh, double current_jd, double mu,
                       double init_time = 0);

  void CalculateSystemMatrix(RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit, double mu);
  void CalculateSTM(STMModel stm_model_type, const Orbit* reference_sat_orbit, double mu, double elapsed_sec);
  void PropagateRK4(double elapsed_sec);
  void PropagateSTM(double elapsed_sec);
};
