/*
 * @file AntennaRadiationPattern.hpp
 * @brief Class to manage antenna radiation pattern
 */

#pragma once
#include <Library/math/Constant.hpp>
#include <vector>

/*
 * @class AntennaRadiationPattern
 * @brief Antenna radiation pattern
 * @details theta = [0, 2pi], theta = 0 is on the plus Z axis
 *          phi = [0, pi], phi = 0 is on the plus X axis, and phi = pi/2 is on the plus Y axis
 */
class AntennaRadiationPattern {
 public:
  /**
   * @fn AntennaRadiationPattern
   * @brief Default Constructor
   */
  AntennaRadiationPattern();

  /**
   * @fn AntennaRadiationPattern
   * @brief Constructor
   * @param[in] file_path: Path to antenna pattern CSV file
   */
  AntennaRadiationPattern(std::string file_path);

  /**
   * @fn ~AntennaRadiationPattern
   * @brief Destructor
   */
  ~AntennaRadiationPattern();

  /**
   * @fn GetGain_dB
   * @brief Get antenna gain [dB]
   * @param[in] theta_rad: Angle for theta direction [rad]
   * @param[in] phi_rad: Angle for phi direction [rad]
   * @return Antenna gain [dB]
   */
  double GetGain_dB(const double theta_rad, const double phi_rad);

 private:
  size_t length_theta_ = 360;                //!< Length of grid for theta direction
  size_t length_phi_ = 181;                  //!< Length of grid for phi direction
  const double theta_max_rad_ = libra::tau;  //!< Maximum value of theta
  const double phi_max_rad_ = libra::pi;     //!< Maximum value of phi

  std::vector<std::vector<double>> gain_dB_;  //!< Antenna gain table
};
