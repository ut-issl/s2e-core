/**
 * @file initialize_monte_carlo_parameters.h
 * @brief Initialized parameters for Monte-Carlo simulation
 */

#ifndef S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_PARAMETERS_H_
#define S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_PARAMETERS_H_

#include <library/math/Quaternion.hpp>
#include <library/math/Vector.hpp>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using libra::Quaternion;
using libra::Vector;

/**
 * @class InitParameter
 * @brief Initialized parameters for Monte-Carlo simulation
 */
class InitParameter {
 public:
  /**
   * @enum RandomizationType
   * @brief Randomization type
   */
  enum RandomizationType {
    NoRandomization,                //!< Output default value
    CartesianUniform,               //!< Random variables following a uniform distribution in Cartesian frame
    CartesianNormal,                //!< Random variables following a normal distribution in Cartesian frame
    CircularNormalUniform,          //!< r follows normal distribution, and θ follows uniform distribution in Circular frame
    CircularNormalNormal,           //!< r and θ follow normal distribution in Circular frame
    SphericalNormalUniformUniform,  //!< r follows normal distribution, and θ and φ follow uniform distribution in Spherical frame
    SphericalNormalNormal,          //!< r and  θ follow normal distribution, and mean vector angle φ follows uniform distribution [0,2*pi]
    QuaternionUniform,              //!< Perfectly Randomized Quaternion
    QuaternionNormal,               //!< Angle from the default quaternion θ follows normal distribution
  };

  /**
   * @fn InitParameter
   * @brief Constructor
   */
  InitParameter();

  // Setter
  /**
   * @fn SetSeed
   * @brief Set seed of randomization. Use time infomation when is_deterministic = false.
   */
  static void SetSeed(unsigned long seed = 0, bool is_deterministic = false);
  /**
   * @fn SetRandomConfig
   * @brief Set randomization parameters
   */
  template <size_t NumElement1, size_t NumElement2>
  void SetRandomConfig(const Vector<NumElement1>& mean_or_min, const Vector<NumElement2>& sigma_or_max, RandomizationType rnd_type);

  // Getter
  /**
   * @fn GetVec
   * @brief Get randomized vector value results
   */
  template <size_t NumElement>
  void GetVec(Vector<NumElement>& dst_vec) const;
  /**
   * @fn GetQuaternion
   * @brief Get randomized quaternion results
   */
  void GetQuaternion(Quaternion& dst_quat) const;
  /**
   * @fn GetDouble
   * @brief Get randomized value results
   */
  void GetDouble(double& dst) const;

  // Calculation
  /**
   * @fn Randomize
   * @brief Randomize values with randomization parameters
   */
  void Randomize();

 private:
  std::vector<double> val_;  //!< Randomized value

  std::vector<double> mean_or_min_;   //!< mean or minimum value. Refer comment in gen_[RandomizationType] function.
  std::vector<double> sigma_or_max_;  //!< standard deviation or maximum value. Refer comment in gen_[RandomizationType] function.

  // For randomization
  RandomizationType rnd_type_;                             //!< Randomization type
  static std::random_device rnd_;                          //!< Non-deterministic random number generator with time information
  static std::mt19937 mt_;                                 //!< Deterministic random number generator
  static std::uniform_real_distribution<>* uniform_dist_;  //!< Uniform random number generator
  static std::normal_distribution<>* normal_dist_;         //!< Normal random number generator

  /**
   * @fn Uniform_1d
   * @brief Generate 1-dimensional uniform distribution random number
   */
  static double Uniform_1d(double lb, double ub);
  /**
   * @fn Normal_1d
   * @brief Generate 1-dimensional normal distribution random number
   */
  static double Normal_1d(double mean, double std);

  // Generate randomized value
  /**
   * @fn gen_NoRandomization
   * @brief Generate randomized value with NoRandomization mode
   */
  void gen_NoRandomization();
  /**
   * @fn gen_CartesianUniform
   * @brief Generate randomized value with CartesianUniform mode
   * @note Support up to three dimensional value
   * @note mean_or_min_[0]: minimum of x   sigma_or_max_[0]: maximum of x
   *       mean_or_min_[1]: minimum of y   sigma_or_max_[1]: maximum of y
   *       mean_or_min_[2]: minimum of z   sigma_or_max_[2]: maximum of z
   */
  void gen_CartesianUniform();
  /**
   * @fn gen_CartesianNormal
   * @brief Generate randomized value with CartesianNormal mode
   * @note Support up to three dimensional value
   * @note mean_or_min_[0]: average of x   sigma_or_max_[0]: sigma of x
   *       mean_or_min_[1]: average of y   sigma_or_max_[1]: sigma of y
   *       mean_or_min_[2]: average of z   sigma_or_max_[2]: sigma of z
   */
  void gen_CartesianNormal();
  /**
   * @fn gen_CircularNormalUniform
   * @brief Generate randomized value with CircularNormalUniform mode
   * @note mean_or_min_[0]: average of r   sigma_or_max_[0]: sigma of r
   *       mean_or_min_[1]: minimum of θ   sigma_or_max_[1]: maximum of θ
   */
  void gen_CircularNormalUniform();
  /**
   * @fn gen_CircularNormalNormal
   * @brief Generate randomized value with CircularNormalNormal mode
   * @details mean_or_min_[0]: average of r   sigma_or_max_[0]: sigma of r
   * @details mean_or_min_[1]: average of θ   sigma_or_max_[1]: sigma of θ
   */
  void gen_CircularNormalNormal();
  /**
   * @fn gen_SphericalNormalUniformUniform
   * @brief Generate randomized value with SphericalNormalUniformUniform mode
   * @note mean_or_min_[0]: average of r   sigma_or_max_[0]: sigma of r
   *       mean_or_min_[1]: minimum of θ   sigma_or_max_[1]: maximum of θ
   *       mean_or_min_[2]: minimum of φ   sigma_or_max_[2]: maximum of φ
   */
  void gen_SphericalNormalUniformUniform();
  /**
   * @fn gen_SphericalNormalNormal
   * @brief Generate randomized value with SphericalNormalNormal mode
   * @note mean_or_min_[0]: average of X    sigma_or_max_[0]: sigma of r
   *       mean_or_min_[1]: average of Y    sigma_or_max_[1]: sigma of θ
   *       mean_or_min_[2]: average of Z    sigma_or_max_[2]: なし
   */
  void gen_SphericalNormalNormal();
  /**
   * @fn gen_QuaternionUniform
   * @brief Generate randomized value with QuaternionUniform mode
   */
  void gen_QuaternionUniform();
  /**
   * @fn gen_QuaternionNormal
   * @brief Generate randomized value with QuaternionNormal mode
   * @note mean_or_min_[0]: NA   sigma_or_max_[0]: standard deviation of θ [rad]
   */
  void gen_QuaternionNormal();

  // Get randomized value
  /**
   * @fn get_CircularNormalUniform
   * @brief Calculate randomized value with CircularNormalUniform mode
   */
  void get_CircularNormalUniform(Vector<2>& dst, double r_mean, double r_sigma, double theta_min, double theta_max);
  /**
   * @fn get_CircularNormalNormal
   * @brief Calculate randomized value with CircularNormalNormal mode
   */
  void get_CircularNormalNormal(Vector<2>& dst, double r_mean, double r_sigma, double theta_mean, double theta_sigma);
  /**
   * @fn get_SphericalNormalUniformUniform
   * @brief Calculate randomized value with SphericalNormalUniformUniform mode
   */
  void get_SphericalNormalUniformUniform(Vector<3>& dst, double r_mean, double r_sigma, double theta_min, double theta_max, double phi_min,
                                         double phi_max);
  /**
   * @fn get_SphericalNormalNormal
   * @brief Calculate randomized value with SphericalNormalNormal mode
   */
  void get_SphericalNormalNormal(Vector<3>& dst, const Vector<3>& mean_vec);
  /**
   * @fn get_QuaternionUniform
   * @brief Calculate randomized value with QuaternionUniform mode
   */
  void get_QuaternionUniform(Quaternion& dst);
  /**
   * @fn get_QuaternionNormal
   * @brief Calculate randomized value with QuaternionNormal mode
   */
  void get_QuaternionNormal(Quaternion& dst, double theta_sigma);
};

template <size_t NumElement1, size_t NumElement2>
void InitParameter::SetRandomConfig(const Vector<NumElement1>& mean_or_min, const Vector<NumElement2>& sigma_or_max,
                                    InitParameter::RandomizationType rnd_type) {
  rnd_type_ = rnd_type;
  mean_or_min_.clear();
  for (size_t i = 0; i < NumElement1; i++) {
    mean_or_min_.push_back(mean_or_min[i]);
  }
  sigma_or_max_.clear();
  for (size_t i = 0; i < NumElement2; i++) {
    sigma_or_max_.push_back(sigma_or_max[i]);
  }
}

template <size_t NumElement>
void InitParameter::GetVec(Vector<NumElement>& dst_vec) const {
  if (rnd_type_ == NoRandomization) {
    ;
  } else if (NumElement > val_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    for (size_t i = 0; i < NumElement; i++) {
      dst_vec[i] = val_[i];
    }
  }
}

#endif  // S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_PARAMETERS_H_
