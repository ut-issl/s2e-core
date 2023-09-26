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
  CelestialInformation(std::string inertial_frame, std::string aber_cor, std::string center_obj, RotationMode rotation_mode, int num_of_selected_body,
                       int* selected_body);
  CelestialInformation(const CelestialInformation& obj);
  virtual ~CelestialInformation();

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Update the all selected celestial objects information
  void UpdateAllObjectsInfo(const double current_jd, const double current_et);

  // Getters
  // Orbit information
  Vector<3> GetPosFromCenter_i(const int id) const;
  Vector<3> GetVelFromCenter_i(const int id) const;
  Vector<3> GetPosFromCenter_i(const char* body_name) const;
  Vector<3> GetVelFromCenter_i(const char* body_name) const;
  // Gravity constants
  double GetGravityConstant(const char* body_name) const;
  double GetCenterBodyGravityConstant_m3_s2(void) const;
  // Shape information
  Vector<3> GetRadii(const int id) const;
  Vector<3> GetRadiiFromName(const char* body_name) const;
  double GetMeanRadiusFromName(const char* body_name) const;
  // Parameters
  inline int GetNumBody(void) const { return num_of_selected_body_; }
  inline int* GetSelectedBody(void) const { return selected_body_; }
  inline std::string GetCenterBodyName(void) const { return center_obj_; }
  // Members
  inline CelestialRotation GetEarthRotation(void) const { return *EarthRotation_; };

  // Calculation
  int CalcBodyIdFromName(const char* body_name) const;
  void DebugOutput(void);

 private:
  // Setting parameters
  int num_of_selected_body_;    //!< number of selected body
  int* selected_body_;          //!< SPICE IDs of selected bodies
  std::string inertial_frame_;  //!< Definition of inertial frame
  std::string aber_cor_;        //!< Stellar aberration correction
                                // （Ref：http://fermi.gsfc.nasa.gov/ssc/library/fug/051108/Aberration_Julie.ppt）
  std::string center_obj_;      //!< Center object of inertial frame

  // Calculated values
  double* celes_objects_pos_from_center_i_;       //!< Position vector list at inertial frame [m]
  double* celes_objects_vel_from_center_i_;       //!< Velocity vector list at inertial frame [m/s]
  double* celes_objects_gravity_constant_;        //!< Gravity constant list [m^3/s^2]
  double* celes_objects_mean_radius_m_;           //!< mean radius list [m] r = (rx * ry * rz)^(1/3)
  double* celes_objects_planetographic_radii_m_;  //!< 3 axis planetographic radii.
                                                  // X-axis pass through the 0 degree latitude 0 degree longitude direction
                                                  // Z-axis pass through the 90 degree latitude direction
                                                  // Y-axis equal to the cross product of the unit Z-axis and X-axis vectors

  // Rotational Motion of each planets
  CelestialRotation* EarthRotation_;
  RotationMode rotation_mode_;  //!< Designation of rotation model

  // Override function of SPICE
  void GetPlanetOrbit(const char* planet_name, double et, double orbit[6]);
};

#endif  //__celestial_information_H__
