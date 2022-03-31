#ifndef __celestial_information_H__
#define __celestial_information_H__

#include <cstring>
#include <string>

#include "CelestialRotation.h"
#include "Interface/LogOutput/ILoggable.h"
#include "Library/math/MatVec.hpp"
#include "Library/math/Matrix.hpp"
#include "Library/math/Quaternion.hpp"
#include "Library/math/Vector.hpp"

using libra::Quaternion;
using libra::Vector;

class CelestialInformation : public ILoggable {
 public:
  // CONSTRUCTOR OF CELESTIAL INFORMATION
  CelestialInformation(std::string inertial_frame, std::string aber_cor, std::string center_obj, RotationMode rotation_mode, int num_of_selected_body,
                       int* selected_body);
  CelestialInformation(const CelestialInformation& obj);
  ~CelestialInformation();

  // UPDATE THE ALL SELECTED CELESTIAL OBJECTS INFORMATION
  void UpdateAllObjectsInfo(const double current_jd);

  // GET FUNCTIONS
  Vector<3> GetPosFromCenter_i(const int id) const;
  Vector<3> GetVelFromCenter_i(const int id) const;
  Vector<3> GetRadii(const int id) const;
  Vector<3> GetPosFromCenter_i(const char* body_name) const;
  Vector<3> GetVelFromCenter_i(const char* body_name) const;
  double GetGravityConstant(const char* body_name) const;
  Vector<3> GetRadiiFromName(const char* body_name) const;
  double GetMeanRadiusFromName(const char* body_name) const;
  inline int GetNumBody(void) const { return num_of_selected_body_; }
  inline int* GetSelectedBody(void) const { return selected_body_; }
  int CalcBodyIdFromName(const char* body_name) const;
  inline std::string GetCenterBodyName(void) const { return center_obj_; }

  inline CelestialRotation GetEarthRotation(void) const { return *EarthRotation_; };

  // FOR LOG OUTPUT
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);

 private:
  int num_of_selected_body_;
  int* selected_body_;          // IDs of selected bodies.
  std::string inertial_frame_;  // Definition of inertial frame. Default = "J2000"
  std::string aber_cor_;        // stellar aberration correction. Default =
                                // "NONE"（Ref：http://fermi.gsfc.nasa.gov/ssc/library/fug/051108/Aberration_Julie.ppt）
  std::string center_obj_;      // center object. Default = "EARTH"
  RotationMode rotation_mode_;  // designation of dynamics model. Default = "Full"

  // Global Information. POS:[m], VEL:[m/s], GRAVITY CONSTANT (G*M):[m^3/s^2]
  double* celes_objects_pos_from_center_i_;
  double* celes_objects_vel_from_center_i_;
  double* celes_objects_gravity_constant_;
  // 3 axis planetographic radii.
  // X-axis pass through the 0 degree latitude 0 degree longitude direction
  // Z-axis pass through the 90 degree latitude direction
  // Y-axis equal to the cross product of the unit Z-axis and X-axis vectors
  double* celes_objects_planetographic_radii_m_;
  // Mean radius: r = (rx * ry * rz)^(1/3)
  double* celes_objects_mean_radius_m_;

  // Rotational Motion of each planets
  CelestialRotation* EarthRotation_;
};
#endif  //__celestial_information_H__
