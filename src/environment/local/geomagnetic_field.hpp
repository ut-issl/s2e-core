/**
 * @file geomagnetic_field.hpp
 * @brief Class to calculate magnetic field of the earth
 */

#ifndef S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_
#define S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_

#include "library/logger/loggable.hpp"
#include "library/math/quaternion.hpp"
#include "library/math/vector.hpp"

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
   * @param [in] igrf_file_name: Path to initialize file
   * @param [in] random_walk_srandard_deviation_nT: Standard deviation of Random Walk [nT]
   * @param [in] random_walk_limit_nT: Limit of Random Walk [nT]
   * @param [in] white_noise_standard_deviation_nT: Standard deviation of white noise [nT]
   */
  MagEnvironment(std::string igrf_file_name, double random_walk_srandard_deviation_nT, double random_walk_limit_nT,
                 double white_noise_standard_deviation_nT);
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
  void CalcMag(double decyear, double side, libra::Vector<3> lat_lon_alt, libra::Quaternion q_i2b);

  /**
   * @fn GetMag_i
   * @brief Return magnetic field vector in the inertial frame [nT]
   */
  libra::Vector<3> GetMag_i() const;
  /**
   * @fn GetMag_b
   * @brief Return magnetic field vector in the body fixed frame [nT]
   */
  libra::Vector<3> GetMag_b() const;

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
  libra::Vector<3> magnetic_field_i_nT_;      //!< Magnetic field vector at the inertial frame [nT]
  libra::Vector<3> magnetic_field_b_nT_;      //!< Magnetic field vector at the spacecraft body fixed frame [nT]
  double random_walk_srandard_deviation_nT_;  //!< Standard deviation of Random Walk [nT]
  double random_walk_limit_nT_;               //!< Limit of Random Walk [nT]
  double white_noise_standard_deviation_nT_;  //!< Standard deviation of white noise [nT]
  std::string igrf_file_name_;                //!< Path to the initialize file

  /**
   * @fn AddNoise
   * @brief Add magnetic field noise
   * @param [in/out] mag_i_array: input true magnetic field, output magnetic field with noise
   */
  void AddNoise(double* mag_i_array);
};

#endif  // S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_
