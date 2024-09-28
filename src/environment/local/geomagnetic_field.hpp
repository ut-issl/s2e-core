/**
 * @file geomagnetic_field.hpp
 * @brief Class to calculate magnetic field of the earth
 */

#ifndef S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_
#define S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_

#include "logger/loggable.hpp"
#include "math_physics/geodesy/geodetic_position.hpp"
#include "math_physics/math/quaternion.hpp"
#include "math_physics/math/vector.hpp"

namespace s2e::environment {

/**
 * @class GeomagneticField
 * @brief Class to calculate magnetic field of the earth
 */
class GeomagneticField : public logger::ILoggable {
 public:
  bool IsCalcEnabled = true;  //!< Calculation flag

  /**
   * @fn GeomagneticField
   * @brief Constructor
   * @param [in] igrf_file_name: Path to initialize file
   * @param [in] random_walk_srandard_deviation_nT: Standard deviation of Random Walk [nT]
   * @param [in] random_walk_limit_nT: Limit of Random Walk [nT]
   * @param [in] white_noise_standard_deviation_nT: Standard deviation of white noise [nT]
   */
  GeomagneticField(const std::string igrf_file_name, const double random_walk_srandard_deviation_nT, const double random_walk_limit_nT,
                   const double white_noise_standard_deviation_nT);
  /**
   * @fn ~GeomagneticField
   * @brief Destructor
   */
  virtual ~GeomagneticField() {}

  /**
   * @fn CalcMagneticField
   * @brief Calculate magnetic field vector
   * @param [in] decimal_year: Decimal year [year]
   * @param [in] sidereal_day: Sidereal day [day]
   * @param [in] position: Position of target point to calculate the magnetic field
   * @param [in] quaternion_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   */
  void CalcMagneticField(const double decimal_year, const double sidereal_day, const s2e::geodesy::GeodeticPosition position,
                         const math::Quaternion quaternion_i2b);

  /**
   * @fn GetGeomagneticField_i_nT
   * @brief Return magnetic field vector in the inertial frame [nT]
   */
  inline math::Vector<3> GetGeomagneticField_i_nT() const { return magnetic_field_i_nT_; }
  /**
   * @fn GetGeomagneticField_b_nT
   * @brief Return magnetic field vector in the body fixed frame [nT]
   */
  inline math::Vector<3> GetGeomagneticField_b_nT() const { return magnetic_field_b_nT_; }

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  math::Vector<3> magnetic_field_i_nT_;       //!< Magnetic field vector at the inertial frame [nT]
  math::Vector<3> magnetic_field_b_nT_;       //!< Magnetic field vector at the spacecraft body fixed frame [nT]
  double random_walk_standard_deviation_nT_;  //!< Standard deviation of Random Walk [nT]
  double random_walk_limit_nT_;               //!< Limit of Random Walk [nT]
  double white_noise_standard_deviation_nT_;  //!< Standard deviation of white noise [nT]
  std::string igrf_file_name_;                //!< Path to the initialize file

  /**
   * @fn AddNoise
   * @brief Add magnetic field noise
   * @param [in/out] magnetic_field_array_i_nT: input true magnetic field, output magnetic field with noise
   */
  void AddNoise(double* magnetic_field_array_i_nT);
};

/**
 * @fn InitGeomagneticField
 * @brief Initialize magnetic field of the earth
 * @param [in] initialize_file_path: Path to initialize file
 */
GeomagneticField InitGeomagneticField(std::string initialize_file_path);

} // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_LOCAL_GEOMAGNETIC_FIELD_HPP_
