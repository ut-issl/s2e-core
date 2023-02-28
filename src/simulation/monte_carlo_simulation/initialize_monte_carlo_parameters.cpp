/**
 * @file initialize_monte_carlo_parameters.cpp
 * @brief Initialized parameters for Monte-Carlo simulation
 */

#include "initialize_monte_carlo_parameters.hpp"

#include <library/math/constants.hpp>

using namespace std;

random_device InitMonteCarloParameters::randomizer_;
mt19937 InitMonteCarloParameters::mt_;
uniform_real_distribution<>* InitMonteCarloParameters::uniform_distribution_;
normal_distribution<>* InitMonteCarloParameters::normal_distribution_;

InitMonteCarloParameters::InitMonteCarloParameters() {
  // Generate object when the first execution
  static bool initial_setup_done = false;
  if (!initial_setup_done) {
    SetSeed();
    InitMonteCarloParameters::uniform_distribution_ = new uniform_real_distribution<>(0.0, 1.0);
    InitMonteCarloParameters::normal_distribution_ = new normal_distribution<>(0.0, 1.0);
    initial_setup_done = true;
  }

  // No randomization when SetRandomConfiguration is not called（No setting in MCSim.ini）
  randomization_type_ = NoRandomization;
}

void InitMonteCarloParameters::SetSeed(unsigned long seed, bool is_deterministic) {
  if (is_deterministic) {
    InitMonteCarloParameters::mt_.seed(seed);
  } else {
    InitMonteCarloParameters::mt_.seed(InitMonteCarloParameters::randomizer_());
  }
}

void InitMonteCarloParameters::GetRandomizedScalar(double& destination) const {
  if (randomization_type_ == NoRandomization) {
    ;
  } else if (1 > randomized_value_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    destination = randomized_value_[0];
  }
}

void InitMonteCarloParameters::GetRandomizedQuaternion(libra::Quaternion& destination) const {
  if (randomization_type_ == NoRandomization) {
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

void InitMonteCarloParameters::Randomize() {
  switch (randomization_type_) {
    case NoRandomization:
      GenerateNoRandomization();
      break;
    case CartesianUniform:
      GenerateCartesianUniform();
      break;
    case CartesianNormal:
      GenerateCartesianNormal();
      break;
    case CircularNormalUniform:
      GenerateCircularNormalUniform();
      break;
    case CircularNormalNormal:
      GenerateCircularNormalNormal();
      break;
    case SphericalNormalUniformUniform:
      GenerateSphericalNormalUniformUniform();
      break;
    case SphericalNormalNormal:
      GenerateSphericalNormalNormal();
      break;
    case QuaternionUniform:
      GenerateQuaternionUniform();
      break;
    case QuaternionNormal:
      GenerateQuaternionNormal();
      break;
    default:
      break;
  }
}

double InitMonteCarloParameters::Generate1dUniform(double lb, double ub) {
  return lb + (*InitMonteCarloParameters::uniform_distribution_)(InitMonteCarloParameters::mt_) * (ub - lb);
}

double InitMonteCarloParameters::Generate1dNormal(double mean, double std) {
  return mean + (*InitMonteCarloParameters::normal_distribution_)(InitMonteCarloParameters::mt_) * (std);
}

void InitMonteCarloParameters::GenerateNoRandomization() { randomized_value_.clear(); }

void InitMonteCarloParameters::GenerateCartesianUniform() {
  // Random variables following a uniform distribution in Cartesian frame
  randomized_value_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    randomized_value_.push_back(InitMonteCarloParameters::Generate1dUniform(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitMonteCarloParameters::GenerateCartesianNormal() {
  // Random variables following a normal distribution in Cartesian frame
  randomized_value_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    randomized_value_.push_back(InitMonteCarloParameters::Generate1dNormal(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitMonteCarloParameters::CalcCircularNormalUniform(libra::Vector<2>& destination, double r_mean, double r_sigma, double theta_min,
                                                         double theta_max) {
  // r follows normal distribution, and θ follows uniform distribution in Circular frame
  double r = InitMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = InitMonteCarloParameters::Generate1dUniform(theta_min, theta_max);
  destination[0] = r * cos(theta);
  destination[1] = r * sin(theta);
}

void InitMonteCarloParameters::GenerateCircularNormalUniform() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  libra::Vector<dim> temp_vec;
  CalcCircularNormalUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitMonteCarloParameters::CalcCircularNormalNormal(libra::Vector<2>& destination, double r_mean, double r_sigma, double theta_mean,
                                                        double theta_sigma) {
  // r and θ follow normal distribution in Circular frame
  double r = InitMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = InitMonteCarloParameters::Generate1dNormal(theta_mean, theta_sigma);
  destination[0] = r * cos(theta);
  destination[1] = r * sin(theta);
}

void InitMonteCarloParameters::GenerateCircularNormalNormal() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  libra::Vector<dim> temp_vec;
  CalcCircularNormalNormal(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitMonteCarloParameters::CalcSphericalNormalUniformUniform(libra::Vector<3>& destination, double r_mean, double r_sigma, double theta_min,
                                                                 double theta_max, double phi_min, double phi_max) {
  // r follows normal distribution, and θ and φ follow uniform distribution in Spherical frame
  double r = InitMonteCarloParameters::Generate1dNormal(r_mean, r_sigma);
  double theta = acos(cos(theta_min) - (cos(theta_min) - cos(theta_max)) * InitMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitMonteCarloParameters::Generate1dUniform(phi_min, phi_max);
  destination[0] = r * sin(theta) * cos(phi);
  destination[1] = r * sin(theta) * sin(phi);
  destination[2] = r * cos(theta);
}

void InitMonteCarloParameters::GenerateSphericalNormalUniformUniform() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  libra::Vector<dim> temp_vec;
  CalcSphericalNormalUniformUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1], mean_or_min_[2],
                                    sigma_or_max_[2]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitMonteCarloParameters::CalcSphericalNormalNormal(libra::Vector<3>& destination, const libra::Vector<3>& mean_vec) {
  // r and  θ follow normal distribution, and mean vector angle φ follows uniform distribution [0,2*pi]
  libra::Vector<3> mean_vec_dir;
  mean_vec_dir = 1.0 / CalcNorm(mean_vec) * mean_vec;  // Unit vector of mean vector direction

  libra::Vector<3> x_axis(0.0), y_axis(0.0);
  x_axis[0] = 1.0;
  y_axis[1] = 1.0;
  libra::Vector<3> op_x = OuterProduct(mean_vec_dir, x_axis);
  libra::Vector<3> op_y = OuterProduct(mean_vec_dir, y_axis);

  // An unit vector perpendicular with the mean vector
  // In case of the mean vector is parallel with X or Y axis, selecting the axis depend on the norm of outer product
  libra::Vector<3> normal_unit_vec = CalcNorm(op_x) > CalcNorm(op_y) ? Normalize(op_x) : Normalize(op_y);

  double rotation_angle_of_normal_unit_vec = InitMonteCarloParameters::Generate1dUniform(0.0, libra::tau);
  libra::Quaternion rotation_of_normal_unit_vec(mean_vec_dir,
                                                -rotation_angle_of_normal_unit_vec);  // Use opposite sign to rotate the vector (not the frame)
  libra::Vector<3> rotation_axis = rotation_of_normal_unit_vec.FrameConversion(normal_unit_vec);  // Axis of mean vector rotation

  double rotation_angle_of_mean_vec = InitMonteCarloParameters::Generate1dNormal(0.0, sigma_or_max_[1]);
  libra::Quaternion rotation_of_mean_vec(rotation_axis, -rotation_angle_of_mean_vec);  // Use opposite sign to rotate the vector (not the frame)
  libra::Vector<3> ret_vec = rotation_of_mean_vec.FrameConversion(mean_vec_dir);       // Complete calculation of the direction

  ret_vec = InitMonteCarloParameters::Generate1dNormal(CalcNorm(mean_vec), sigma_or_max_[0]) * ret_vec;  // multiply norm

  for (int i = 0; i < 3; i++) {
    destination[i] = ret_vec[i];
  }
}

void InitMonteCarloParameters::GenerateSphericalNormalNormal() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < 2) {
    throw "Config parameters dimension unmatched.";
  }
  libra::Vector<dim> temp_vec, temp_mean_vec;
  for (int i = 0; i < dim; i++) {
    temp_mean_vec[i] = mean_or_min_[i];
  }
  CalcSphericalNormalNormal(temp_vec, temp_mean_vec);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_vec[i]);
  }
}

void InitMonteCarloParameters::CalcQuaternionUniform(libra::Quaternion& destination) {
  // Perfectly Randomized libra::Quaternion
  libra::Vector<3> x_axis(0.0);
  x_axis[0] = 1.0;

  // A direction vector converted from the X-axis by a quaternion may follows the uniform distribution in full sphere.
  libra::Quaternion first_cnv;
  libra::Vector<3> x_axis_cnvd;
  double theta = acos(1 - (1 - (-1)) * InitMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitMonteCarloParameters::Generate1dUniform(0, libra::tau);
  x_axis_cnvd[0] = sin(theta) * cos(phi);
  x_axis_cnvd[1] = sin(theta) * sin(phi);
  x_axis_cnvd[2] = cos(theta);

  double cos_angle_between = InnerProduct(x_axis, x_axis_cnvd);
  libra::Vector<3> op = OuterProduct(x_axis, x_axis_cnvd);
  for (int i = 0; i < 3; i++) {
    first_cnv[i] = op[i];
  }
  first_cnv[3] = cos_angle_between;

  // Generate randomized rotation angle around the X-axis
  double rotation_angle = InitMonteCarloParameters::Generate1dUniform(0.0, libra::tau);
  libra::Quaternion second_cnv(x_axis, rotation_angle);

  libra::Quaternion ret_q = first_cnv * second_cnv;

  for (int i = 0; i < 4; i++) {
    destination[i] = ret_q[i];
  }
}

void InitMonteCarloParameters::GenerateQuaternionUniform() {
  const static int dim = 4;
  libra::Quaternion temp_q;
  CalcQuaternionUniform(temp_q);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_q[i]);
  }
}

void InitMonteCarloParameters::CalcQuaternionNormal(libra::Quaternion& destination, double theta_sigma) {
  // Angle from the default quaternion θ follows normal distribution
  // The rotation axis follows uniform distribution on full sphere
  libra::Vector<3> rot_axis;
  double theta = acos(1 - (1 - (-1)) * InitMonteCarloParameters::Generate1dUniform(0.0, 1.0));
  double phi = InitMonteCarloParameters::Generate1dUniform(0, libra::tau);
  rot_axis[0] = sin(theta) * cos(phi);
  rot_axis[1] = sin(theta) * sin(phi);
  rot_axis[2] = cos(theta);

  double rotation_angle = InitMonteCarloParameters::Generate1dNormal(0.0, theta_sigma);

  libra::Quaternion ret_q(rot_axis, rotation_angle);
  for (int i = 0; i < 4; i++) {
    destination[i] = ret_q[i];
  }
}

void InitMonteCarloParameters::GenerateQuaternionNormal() {
  // mean_or_min_[0]: NA   sigma_or_max_[0]: standard deviation of θ [rad]
  const static int dim = 4;
  if (sigma_or_max_.size() < 1) {
    throw "Config parameters dimension unmatched.";
  }
  libra::Quaternion temp_q;
  CalcQuaternionNormal(temp_q, sigma_or_max_[0]);

  randomized_value_.clear();
  for (int i = 0; i < dim; i++) {
    randomized_value_.push_back(temp_q[i]);
  }
}
