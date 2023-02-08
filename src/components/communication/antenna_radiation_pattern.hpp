/*
 * @file antenna_radiation_pattern.hpp
 * @brief Class to manage antenna radiation pattern
 */

#ifndef S2E_COMPONENTS_COMMUNICATION_ANTENNA_RADIATION_PATTERN_H_
#define S2E_COMPONENTS_COMMUNICATION_ANTENNA_RADIATION_PATTERN_H_

#include <Library/math/Constant.hpp>
#include <string>
#include <vector>

/*
 * @class AntennaRadiationPattern
 * @brief Antenna radiation pattern
 * @details theta = [0, 2pi], theta = 0 is on the plus Z axis
 *          phi = [0, pi], phi = 0 is on the plus X axis, and phi = pi/2 is on the plus Y axis
 *          The unit of gain values in the CSV file should be [dBi]
 */
class AntennaRadiationPattern {
 public:
  /**
   * @fn AntennaRadiationPattern
   * @brief Default Constructor
   * @note The gain table is initialized as all zero value
   */
  AntennaRadiationPattern();

  /**
   * @fn AntennaRadiationPattern
   * @brief Constructor
   * @param[in] file_path: Path to antenna pattern CSV file
   * @param[in] length_theta_: Length of grid for theta direction
   * @param[in] length_phi_: Length of grid for phi direction
   * @param[in] theta_max_rad_: Maximum value of theta
   * @param[in] phi_max_rad_: Maximum value of phi
   */
  AntennaRadiationPattern(const std::string file_path, const size_t length_theta = 360, const size_t length_phi = 181,
                          const double theta_max_rad = libra::tau, const double phi_max_rad = libra::pi);

  /**
   * @fn ~AntennaRadiationPattern
   * @brief Destructor
   */
  ~AntennaRadiationPattern();

  /**
   * @fn GetGain_dBi
   * @brief Get antenna gain [dBi]
   * @param[in] theta_rad: theta = [0, max_theta], theta = 0 is on the plus Z axis
   * @param[in] phi_rad: phi = [0, max_phi], phi = 0 is on the plus X axis, and phi = pi/2 is on the plus Y axis
   * @return Antenna gain [dBi]
   */
  double GetGain_dBi(const double theta_rad, const double phi_rad) const;

 private:
  size_t length_theta_ = 360;          //!< Length of grid for theta direction
  size_t length_phi_ = 181;            //!< Length of grid for phi direction
  double theta_max_rad_ = libra::tau;  //!< Maximum value of theta
  double phi_max_rad_ = libra::pi;     //!< Maximum value of phi

  std::vector<std::vector<double>> gain_dBi_;  //!< Antenna gain table [dBi]
};

#endif  // S2E_COMPONENTS_COMMUNICATION_ANTENNA_RADIATION_PATTERN_H_
