#ifndef __controlled_attitude_H__
#define __controlled_attitude_H__

#include <string>
using namespace std;

#include "Attitude.h"
#include "../../Environment/Local/LocalCelestialInformation.h"
#include "../Orbit/Orbit.h"

class ControlledAttitude : public Attitude
{
public:
  ControlledAttitude(
    const AttCtrlMode main_mode,
    const AttCtrlMode sub_mode,
    const Quaternion quaternion_i2t,
    const Vector<3> pointing_t_b,
    const Vector<3> pointing_sub_t_b,
    const LocalCelestialInformation* local_celes_info,
    const Orbit* orbit
  );
  ~ControlledAttitude();
  //Main functions
  void Initialize(void);
  virtual void Propagate(double endtime);

  // Override ILoggable
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  //Inputs
  const LocalCelestialInformation* local_celes_info_;
  const Orbit* orbit_;
  //Local functions
  Vector<3> CalcTargetDirection(AttCtrlMode mode);
  void PointingCtrl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i);
  Matrix<3,3> CalcDCM(const Vector<3> main_direction, const Vector<3> sub_direction);
};

#endif //__controlled_attitude_H__
