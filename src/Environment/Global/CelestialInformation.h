#ifndef __celestial_information_H__
#define __celestial_information_H__

#include <cstring>
#include <string>
using namespace std;

#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Interface/LogOutput/ILoggable.h"

using libra::Vector;
using libra::Quaternion;

class CelestialInformation : public ILoggable
{
public:
  // CONSTRUCTOR OF CELESTIAL INFORMATION
  CelestialInformation(string inertial_frame, string aber_cor, string center_obj, int num_of_selected_body, int* selected_body);
  CelestialInformation(const CelestialInformation &obj);
  ~CelestialInformation();

  // UPDATE THE ALL SELECTED CELESTIAL OBJECTS INFORMATION
  void UpdateAllObjectsInfo(const double current_jd);

  // GET FUNCTIONS
  Vector<3> GetPosFromCenter_i(const int id) const;
  Vector<3> GetVelFromCenter_i(const int id) const;
  Vector<3> GetPosFromCenter_i(const char* body_name) const;
  Vector<3> GetVelFromCenter_i(const char* body_name) const;
  double GetGravityConstant(const char* body_name) const;
  inline int GetNumBody(void) const{return num_of_selected_body_;}
  inline int* GetSelectedBody(void) const{return selected_body_;}
  int CalcBodyIdFromName(const char* body_name) const;
  inline string GetCenterBodyName(void) const { return center_obj_; }

  // FOR LOG OUTPUT
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);

private:
  int   num_of_selected_body_;
  int*  selected_body_;       //IDs of selected bodies.
  string inertial_frame_;     //Definition of inertial frame. Default = "J2000"
  string aber_cor_;           //stellar aberration correction. Default = "NONE"（Ref：http://fermi.gsfc.nasa.gov/ssc/library/fug/051108/Aberration_Julie.ppt）
  string center_obj_;         //center object. Default = "EARTH"

  // Global Information. POS:[m], VEL:[m/s], GRAVITY CONSTANT (G*M):[m^3/s^2]
  double* celes_objects_pos_from_center_i_;
  double* celes_objects_vel_from_center_i_;
  double* celes_objects_gravity_constant_;
};
#endif //__celestial_information_H__
