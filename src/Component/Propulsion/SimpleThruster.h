#pragma once

#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Quaternion.hpp"

#include "../Abstract/ComponentBase.h"
#include "../../Dynamics/Dynamics.h"
#include "../../Simulation/Spacecraft/Structure/Structure.h"
#include "../../Interface/LogOutput/Logger.h"

class SimpleThruster : public ComponentBase, public ILoggable
{
public:
  SimpleThruster(
    const int prescaler,
    ClockGenerator* clock_gen,
    const int id, 
    const Vector<3> thruster_pos_b, 
    const Vector<3> thrust_dir_b, 
    const double max_mag,   // N
    const double mag_err,   // N
    const double dir_err,   // rad
    const Structure* structure, 
    const Dynamics* dynamics
  );
  SimpleThruster(
    const int prescaler,
    ClockGenerator* clock_gen,
    PowerPort* power_port,
    const int id, 
    const Vector<3> thruster_pos_b, 
    const Vector<3> thrust_dir_b, 
    const double max_mag,   // N
    const double mag_err,   // N
    const double dir_err,   // rad
    const Structure* structure, 
    const Dynamics* dynamics
  );
  ~SimpleThruster();

  // ComponentBase override function
  void MainRoutine(int count);

  // ILogabble override function
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline const Vector<3> GetThrustB() const { return thrust_b_; };
  inline const Vector<3> GetTorqueB()const { return torque_b_; };

  // Setter
  inline void SetDuty(double duty){duty_=duty;};

protected:
  // parameters
  const int id_;
  Vector<3> thruster_pos_b_{0.0};   // Thruster position @ body frame
  Vector<3> thrust_dir_b_{0.0};     // Thrust direction @ body frame
  double duty_=0.0; // [0.0:1.0]
  double thrust_magnitude_max_=0.0; // N
  double thrust_dir_err_=0.0; // Thrust direction error rad
  libra::NormalRand mag_nr_, dir_nr_; 
  // outputs
  Vector<3> thrust_b_{0.0};
  Vector<3> torque_b_{0.0};

  void CalcThrust();
  void CalcTorque(Vector<3> center, double temp);
  double CalcThrustMagnitude();
  Vector<3> CalcThrustDir();
  void Initialize(const double mag_err, const double dir_err);

  const Structure* structure_;
  const Dynamics* dynamics_;
};
