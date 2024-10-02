/**
 * @file initialize_monte_carlo_parameters.cpp
 * @brief Initialized parameters for Monte-Carlo simulation
 */

#include "initialize_monte_carlo_parameters.hpp"

#include <math_physics/math/constants.hpp>

using namespace std;

namespace s2e::simulation {

random_device InitializedMonteCarloParameters::randomizer_;
mt19937 InitializedMonteCarloParameters::mt_;
uniform_real_distribution<>* InitializedMonteCarloParameters::uniform_distribution_;
normal_distribution<>* InitializedMonteCarloParameters::normal_distribution_;

InitializedMonteCarloParameters::InitializedMonteCarloParameters() {
  // Generate object when the first execution
  static bool initial_setup_done = false;
  if (!initial_setup_done) {
    SetSeed();
    InitializedMonteCarloParameters::uniform_distribution_ = new uniform_real_distribution<>(0.0, 1.0);
    InitializedMonteCarloParameters::normal_distribution_ = new normal_distribution<>(0.0, 1.0);
    initial_setup_done = true;
  }

  // No randomization when SetRandomConfiguration is not called（No setting in MCSim.ini）
  randomization_type_ = kNoRandomization;
}

void InitializedMonteCarloParameters::SetSeed(unsigned long seed, bool is_deterministic) {
  if (is_deterministic) {
    InitializedMonteCarloParameters::mt_.seed(seed);
  } else {
    InitializedMonteCarloParameters::mt_.seed(InitializedMonteCarloParameters::randomizer_());
  }
}

void InitializedMonteCarloParameters::GetRandomizedScalar(double& destination) const {
  if (randomization_type_ == kNoRandomization) {
    ;
  } else if (1 > randomized_value_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    destination = randomized_value_[0];
  }
}

void InitializedMonteCarloParameters::GetRandomizedQuaternion(math::Quaternion& destination) const {
  if (randomization_type_ == kNoRandomization) {
    ;
  } else if (4 > randomized_value_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    for (int i = 0; i < 4; i++) {
      destination[i] = randomized_value_[i];
    }
  }

  destination.Normalize();
}

void InitializedMonteCarloParameters::Randomize() {
  switch (randomization_type_) {
    case kNoRandomization:
      GenerateNoRandomization();
      break;
    case kCartesianUniform:
      GenerateCartesianUniform();
      break;
    case kCartesianNormal:
      GenerateCartesianNormal();
      break;
    case kCircularNormalUniform:
      GenerateCircularNormalUniform();
      break;
    case kCircularNormalNormal:
      GenerateCircularNormalNormal();
      break;
    case kSphericalNormalUniformUniform:
      GenerateSphericalNormalUniformUniform();
      break;
    case kSphericalNormalNormal:
      GenerateSphericalNormalNormal();
      break;
    case kQuaternionUniform:
      GenerateQuaternionUniform();
      break;
    case kQuaternionNormal:
      GenerateQuaternionNormal();
      break;
    default:
      break;
  }
}

double InitializedMonteCarloParameters::Generate1dUniform(double lb, double ub) {
  return lb + (*InitializedMonteCarloParameters::uniform_distribution_)(InitializedMonteCarloParameters::mt_) * (ub - lb);
}

double InitializedMonteCarloParameters::Generate1dNormal(double mean, double std) {
  return mean + (*InitializedMonteCarloParameters::normal_distribution_)(InitializedMonteCarloParameters::mt_) * (std);
}

void InitializedMonteCarloParameters::GenerateNoRandomization() { randomized_value_.clear(); }

void InitializedMonteCarloParameters::GenerateCartesianUniform() {
  // Random variables following a uniform distribution in Cartesian frame
  randomized_value_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    randomized_value_.push_back(InitializedMonteCarloParameters::Generate1dUniform(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitializedMonteCarloParameters::GenerateCartesianNormal() {
  // Random variables following a normal distribution in Cartesian frame
  randomized_value_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    randomized_value_.push_back(InitializedMonteCarloParameters::Generate1dNormal(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitializedMonteCarloParameters::CalcCircularNormalUniform(math::Vector<2>& destination, double r_mean, double r_sigma, double theta_min,
                                                                double theta_max) {
  // r follows normal distribution, and θ follows uniform distribution in Circular frame
  double r = InitializedMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = InitializedMonteCarloParameters::Generate1dUniform(theta_min, theta_max);
  destination[0] = r * cos(theta);
  destination[1] = r * sin(theta);
}

void InitializedMonteCarloParameters::GenerateCircularNormalUniform() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  math::Vector<dim> temp_vec;
  CalcCircularNormalUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitializedMonteCarloParameters::CalcCircularNormalNormal(math::Vector<2>& destination, double r_mean, double r_sigma, double theta_mean,
                                                               double theta_sigma) {
  // r and θ follow normal distribution in Circular frame
  double r = InitializedMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = InitializedMonteCarloParameters::Generate1dNormal(theta_mean, theta_sigma);
  destination[0] = r * cos(theta);
  destination[1] = r * sin(theta);
}

void InitializedMonteCarloParameters::GenerateCircularNormalNormal() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  math::Vector<dim> temp_vec;
  CalcCircularNormalNormal(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitializedMonteCarloParameters::CalcSphericalNormalUniformUniform(math::Vector<3>& destination, double r_mean, double r_sigma, double theta_min,
                                                                        double theta_max, double phi_min, double phi_max) {
  // r follows normal distribution, and θ and φ follow uniform distribution in Spherical frame
  double r = InitializedMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = acos(cos(theta_min) - (cos(theta_min) - cos(theta_max)) * InitializedMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitializedMonteCarloParameters::Generate1dUniform(phi_min, phi_max);
  destination[0] = r * sin(theta) * cos(phi);
  destination[1] = r * sin(theta) * sin(phi);
  destination[2] = r * cos(theta);
}

void InitializedMonteCarloParameters::GenerateSphericalNormalUniformUniform() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  math::Vector<dim> temp_vec;
  CalcSphericalNormalUniformUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1], mean_or_min_[2],
                                    sigma_or_max_[2]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitializedMonteCarloParameters::CalcSphericalNormalNormal(math::Vector<3>& destination, const math::Vector<3>& mean_vec) {
  // r and  θ follow normal distribution, and mean vector angle φ follows uniform distribution [0,2*pi]
  math::Vector<3> mean_vec_dir;
  mean_vec_dir = 1.0 / mean_vec.CalcNorm() * mean_vec;  // Unit vector of mean vector direction

  math::Vector<3> x_axis(0.0), y_axis(0.0);
  x_axis[0] = 1.0;
  y_axis[1] = 1.0;
  math::Vector<3> op_x = OuterProduct(mean_vec_dir, x_axis);
  math::Vector<3> op_y = OuterProduct(mean_vec_dir, y_axis);

  // An unit vector perpendicular with the mean vector
  // In case of the mean vector is parallel with X or Y axis, selecting the axis depend on the norm of outer product
  math::Vector<3> normal_unit_vec = op_x.CalcNorm() > op_y.CalcNorm() ? op_x = op_x.CalcNormalizedVector() : op_y = op_y.CalcNormalizedVector();

  double rotation_angle_of_normal_unit_vec = InitializedMonteCarloParameters::Generate1dUniform(0.0, math::tau);
  math::Quaternion rotation_of_normal_unit_vec(mean_vec_dir,
                                               -rotation_angle_of_normal_unit_vec);  // Use opposite sign to rotate the vector (not the frame)
  math::Vector<3> rotation_axis = rotation_of_normal_unit_vec.FrameConversion(normal_unit_vec);  // Axis of mean vector rotation

  double rotation_angle_of_mean_vec = InitializedMonteCarloParameters::Generate1dNormal(0.0, sigma_or_max_[1]);
  math::Quaternion rotation_of_mean_vec(rotation_axis, -rotation_angle_of_mean_vec);  // Use opposite sign to rotate the vector (not the frame)
  math::Vector<3> ret_vec = rotation_of_mean_vec.FrameConversion(mean_vec_dir);       // Complete calculation of the direction

  ret_vec = InitializedMonteCarloParameters::Generate1dNormal(mean_vec.CalcNorm(), sigma_or_max_[0]) * ret_vec;  // multiply norm

  for (int i = 0; i < 3; i++) {
    destination[i] = ret_vec[i];
  }
}

void InitializedMonteCarloParameters::GenerateSphericalNormalNormal() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < 2) {
    throw "Config parameters dimension unmatched.";
  }
  math::Vector<dim> temp_vec, temp_mean_vec;
  for (int i = 0; i < dim; i++) {
    temp_mean_vec[i] = mean_or_min_[i];
  }
  CalcSphericalNormalNormal(temp_vec, temp_mean_vec);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitializedMonteCarloParameters::CalcQuaternionUniform(math::Quaternion& destination) {
  // Perfectly Randomized math::Quaternion
  math::Vector<3> x_axis(0.0);
  x_axis[0] = 1.0;

  // A direction vector converted from the X-axis by a quaternion may follows the uniform distribution in full sphere.
  math::Quaternion first_cnv;
  math::Vector<3> x_axis_cnvd;
  double theta = acos(1 - (1 - (-1)) * InitializedMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitializedMonteCarloParameters::Generate1dUniform(0, math::tau);
  x_axis_cnvd[0] = sin(theta) * cos(phi);
  x_axis_cnvd[1] = sin(theta) * sin(phi);
  x_axis_cnvd[2] = cos(theta);

  double cos_angle_between = InnerProduct(x_axis, x_axis_cnvd);
  math::Vector<3> op = OuterProduct(x_axis, x_axis_cnvd);
  for (int i = 0; i < 3; i++) {
    first_cnv[i] = op[i];
  }
  first_cnv[3] = cos_angle_between;

  // Generate randomized rotation angle around the X-axis
  double rotation_angle = InitializedMonteCarloParameters::Generate1dUniform(0.0, math::tau);
  math::Quaternion second_cnv(x_axis, rotation_angle);

  math::Quaternion ret_q = first_cnv * second_cnv;

  for (int i = 0; i < 4; i++) {
    destination[i] = ret_q[i];
  }
}

void InitializedMonteCarloParameters::GenerateQuaternionUniform() {
  const static int dim = 4;
  math::Quaternion temp_q;
  CalcQuaternionUniform(temp_q);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_q[i]);
  }
}

void InitializedMonteCarloParameters::CalcQuaternionNormal(math::Quaternion& destination, double theta_sigma) {
  // Angle from the default quaternion θ follows normal distribution
  // The rotation axis follows uniform distribution on full sphere
  math::Vector<3> rot_axis;
  double theta = acos(1 - (1 - (-1)) * InitializedMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitializedMonteCarloParameters::Generate1dUniform(0, math::tau);
  rot_axis[0] = sin(theta) * cos(phi);
  rot_axis[1] = sin(theta) * sin(phi);
  rot_axis[2] = cos(theta);

  double rotation_angle = InitializedMonteCarloParameters::Generate1dNormal(0.0, theta_sigma);

  math::Quaternion ret_q(rot_axis, rotation_angle);
  for (int i = 0; i < 4; i++) {
    destination[i] = ret_q[i];
  }
}

void InitializedMonteCarloParameters::GenerateQuaternionNormal() {
  // mean_or_min_[0]: NA   sigma_or_max_[0]: standard deviation of θ [rad]
  const static int dim = 4;
  if (sigma_or_max_.size() < 1) {
    throw "Config parameters dimension unmatched.";
  }
  math::Quaternion temp_q;
  CalcQuaternionNormal(temp_q, sigma_or_max_[0]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_q[i]);
  }
}

}  // namespace s2e::simulation
