#ifndef __celestial_information_H__
#define __celestial_information_H__

#include <cstring>
#include <string>
using namespace std;

#include "../Library/math/Matrix.hpp"
#include "../Library/math/Vector.hpp"
#include "../Library/math/MatVec.hpp"
#include "../Library/math/Quaternion.hpp"
#include "../Interface/LogOutput/ILoggable.h"
#include "SpiceUsr.h"

using libra::Vector;
using libra::Quaternion;

class CelestialInformation : public ILoggable
{
private:
  int   num_of_selected_body_;
  int*  selected_body_; //情報取得が必要な天体．.iniファイルで選択
  string inertial_frame_; //慣性座標系の定義．現在のデフォルトは"J2000"
  string aber_cor_; //stellar aberration correction.観測者の速度による天体位置のずれに対する補正の有無．デフォルトは"NONE"．（参照：http://fermi.gsfc.nasa.gov/ssc/library/fug/051108/Aberration_Julie.ppt）
  string center_obj_; //center object. 中心天体．現在のデフォルトは"EARTH"

  // 天体情報．POS：[m], VEL: [m/s]. GRAVITY CONSTANT (G*M): [m^3/s^2]
  double* celes_objects_pos_from_center_i_;
  double* celes_objects_pos_from_center_b_;
  double* celes_objects_pos_from_sc_i_;
  double* celes_objects_pos_from_sc_b_;
  double* celes_objects_vel_from_center_i_;
  double* celes_objects_vel_from_center_b_;
  double* celes_objects_vel_from_sc_i_;
  double* celes_objects_vel_from_sc_b_;
  double* celes_objects_gravity_constant_;

public:
  // CONSTRUCTOR OF CELESTIAL INFORMATION
  CelestialInformation(string inertial_frame, string aber_cor, string center_obj, int num_of_selected_body, int* selected_body);
  CelestialInformation(const CelestialInformation &obj);
  ~CelestialInformation();

  // UPDATE THE ALL SELECTED CELESTIAL OBJECTS INFORMATION
  void UpdateAllObjectsInfo(double current_jd, Vector<3> sc_pos_from_center_i, Vector<3> sc_vel_from_center_i, Quaternion q_i2b, Vector<3> sc_body_rate);
  // FRAME CONVERSION OF ALL BODIES
  void CalcAllPosVel_b(Quaternion q_i2b, Vector<3> sc_body_rate);
  // GET POSITION OF A SELECTED BODY (ORIGIN: S/C, IN INERTIAL FRAME) *NO CALCULATION
  Vector<3> GetPosFromSC_i(const char* body_name) const;
  // GET POSITION OF A SELECTED BODY (ORIGIN: S/C, IN S/C BODY FRAME) *NO CALCULATION
  Vector<3> GetPosFromSC_b(const char* body_name) const;
  // GET POSITION OF A SELECTED BODY FROM SPICE LIBRARY (ORIGIN: S/C, IN INERTIAL FRAME) *WITH CALCULATION
  Vector<6> FetchPosVel(int planet_id, double current_jd);
  // GET POSITION OF A SELECTED BODY FROM SPICE LIBRARY (ORIGIN: S/C, IN INERTIAL FRAME) *WITH CALCULATION
  Vector<6> FetchPosVel(char* planet_name, double current_jd);

  // GET GRAVITY CONSTANT OF SELECTED CELESTIAL BODY
  double GetGravityConstant(const char* body_name);

  // FOR LOG OUTPUT
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);
};
void Convert_i2b(const double* src_i, double* dst_b, const Quaternion q_i2b);

// subroutine for velocity vector conversion
void Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b, const Quaternion q_i2b, const Vector<3> bodyrate_b);

#endif //__celestial_information_H__
