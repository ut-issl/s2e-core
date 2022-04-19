#include <Environment/Global/CelestialInformation.h>

#include <Library/math/ODE.hpp>

#include "Orbit.h"

class Rk4OrbitPropagation : public Orbit, public libra::ODE<6> {
 private:
  static const int N = 6;
  double mu;

 public:
  Rk4OrbitPropagation(const CelestialInformation* celes_info, double mu, double timestep, Vector<3> init_position, Vector<3> init_velocity,
                      double current_jd, double init_time = 0);
  ~Rk4OrbitPropagation();

  virtual void RHS(double t, const Vector<N>& state, Vector<N>& rhs);

  void Initialize(Vector<3> init_position, Vector<3> init_velocity, double current_jd, double init_time = 0);

  virtual void Propagate(double endtime, double current_jd);

  virtual void AddPositionOffset(Vector<3> offset_i);

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  double prop_time_;  // Simulation current time for numerical integration by
                      // RK4
  double prop_step_;  //Δt for RK4
};
