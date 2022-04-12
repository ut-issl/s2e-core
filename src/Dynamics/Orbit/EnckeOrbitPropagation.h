#pragma once
#include "../../Library/Orbit/KeplerOrbit.h"
#include "../../Library/math/ODE.hpp"
#include "Orbit.h"

class EnckeOrbitPropagation : public Orbit, public libra::ODE<6> {
 public:
  EnckeOrbitPropagation(const double mu_m3_s2, const double prop_step_s, const double current_jd, const Vector<3> init_position_i_m,
                        const Vector<3> init_velocity_i_m_s, const double error_tolerance, const int wgs);
  ~EnckeOrbitPropagation();

  // Orbit class
  virtual void Propagate(double endtime, double current_jd);
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // ODE class
  virtual void RHS(double t, const Vector<6>& state, Vector<6>& rhs);

 private:
  // General
  const double mu_m3_s2_;
  const double error_tolerance_;  // error ratio
  double prop_step_s_;            // Δt for RK4
  double prop_time_s_;            // Simulation current time for numerical integration by RK4

  // reference orbit
  Vector<3> ref_position_i_m_;
  Vector<3> ref_velocity_i_m_s_;
  KeplerOrbit ref_kepler_orbit;

  // difference orbit
  Vector<3> diff_position_i_m_;
  Vector<3> diff_velocity_i_m_s_;

  // functions
  void Initialize(double current_jd, Vector<3> init_ref_position_i_m, Vector<3> init_ref_velocity_i_m_s);
  void UpdateSatOrbit(double current_jd);
  double CalcQFunction(Vector<3> diff_pos_i);
};
