#ifndef __local_celestial_information_H__
#define __local_celestial_information_H__

#include "../Global/CelestialInformation.h"

class LocalCelestialInformation : public ILoggable {
 public:
  LocalCelestialInformation(const CelestialInformation* glo_celes_info);
  virtual ~LocalCelestialInformation();

  // UPDATE THE ALL SELECTED CELESTIAL OBJECTS INFORMATION
  void UpdateAllObjectsInfo(const Vector<3> sc_pos_from_center_i, const Vector<3> sc_vel_from_center_i, Quaternion q_i2b,
                            const Vector<3> sc_body_rate);
  // FRAME CONVERSION OF ALL BODIES
  void CalcAllPosVel_b(Quaternion q_i2b, const Vector<3> sc_body_rate);
  // GET POSITION OF A SELECTED BODY (ORIGIN: S/C, IN INERTIAL FRAME)
  Vector<3> GetPosFromSC_i(const char* body_name) const;
  // GET POSITION OF THE CENTER BODY (ORIGIN: S/C, IN INERTIAL FRAME)
  Vector<3> GetCenterBodyPosFromSC_i(void) const;

  // GET POSITION OF A SELECTED BODY (ORIGIN: S/C, IN S/C BODY FRAME)
  Vector<3> GetPosFromSC_b(const char* body_name) const;
  // GET POSITION OF THE CENTER BODY (ORIGIN: S/C, IN S/C BODY FRAME)
  Vector<3> GetCenterBodyPosFromSC_b(void) const;

  // Get Global CelesInfo for gravitational constant
  inline const CelestialInformation& GetGlobalInfo() const { return *glo_celes_info_; }
  // FOR LOG OUTPUT
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  const CelestialInformation* glo_celes_info_;
  // Local Info
  double* celes_objects_pos_from_sc_i_;
  double* celes_objects_vel_from_sc_i_;
  double* celes_objects_pos_from_center_b_;
  double* celes_objects_pos_from_sc_b_;
  double* celes_objects_vel_from_center_b_;
  double* celes_objects_vel_from_sc_b_;
};
void Convert_i2b(const double* src_i, double* dst_b, const Quaternion q_i2b);

// subroutine for velocity vector conversion
void Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b, const Quaternion q_i2b, const Vector<3> bodyrate_b);

#endif  //__local_celestial_information_H__
