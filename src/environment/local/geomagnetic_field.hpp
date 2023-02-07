/**
 * @file geomagnetic_field.hpp
 * @brief Class to calculate magnetic field of the earth
 */

#ifndef S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_H_
#define S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_H_

#include <Library/math/Vector.hpp>
using libra::Vector;
#include <Library/math/Quaternion.hpp>
using libra::Quaternion;

#include <interface/LogOutput/ILoggable.h>

/**
 * @class MagEnvironment
 * @brief Class to calculate magnetic field of the earth
 */
class MagEnvironment : public ILoggable {
 public:
  bool IsCalcEnabled = true;  //!< Calculation flag

  /**
   * @fn MagEnvironment
   * @brief Constructor
   * @param [in] fname: Path to initialize file
   * @param [in] mag_rwdev: Standard deviation of Random Walk [nT]
   * @param [in] mag_rwlimit: Limit of Random Walk [nT]
   * @param [in] mag_wnvar: Standard deviation of white noise [nT]
   */
  MagEnvironment(std::string fname, double mag_rwdev, double mag_rwlimit, double mag_wnvar);
  /**
   * @fn ~MagEnvironment
   * @brief Destructor
   */
  virtual ~MagEnvironment() {}

  /**
   * @fn CalcMag
   * @brief Calculate magnetic field vector
   * @param [in] decyear: Decimal year [year]
   * @param [in] side: Sidereal day [day]
   * @param [in] lat_lon_alt: Latitude [rad], longitude [rad], and altitude [m]
   * @param [in] q_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   */
  void CalcMag(double decyear, double side, Vector<3> lat_lon_alt, Quaternion q_i2b);

  /**
   * @fn GetMag_i
   * @brief Return magnetic field vector in the inertial frame [nT]
   */
  Vector<3> GetMag_i() const;
  /**
   * @fn GetMag_b
   * @brief Return magnetic field vector in the body fixed frame [nT]
   */
  Vector<3> GetMag_b() const;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  Vector<3> Mag_i_;     //!< Magnetic field vector at the inertial frame
  Vector<3> Mag_b_;     //!< Magnetic field vector at the spacecraft body fixed frame
  double mag_rwdev_;    //!< Standard deviation of Random Walk [nT]
  double mag_rwlimit_;  //!< Limit of Random Walk [nT]
  double mag_wnvar_;    //!< Standard deviation of white noise [nT]
  std::string fname_;   //!< Path to the initialize file

  /**
   * @fn AddNoise
   * @brief Add magnetic field noise
   * @param [in/out] mag_i_array: input true magnetic field, output magnetic field with noise
   */
  void AddNoise(double* mag_i_array);
};

#endif  // S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_H_
