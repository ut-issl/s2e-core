/**
 * @file initialize_monte_carlo_parameters.cpp
 * @brief Initialized parameters for Monte-Carlo simulation
 */

#include "initialize_monte_carlo_parameters.hpp"

#include <library/math/Constant.hpp>

using namespace std;

random_device InitParameter::rnd_;
mt19937 InitParameter::mt_;
uniform_real_distribution<>* InitParameter::uniform_dist_;
normal_distribution<>* InitParameter::normal_dist_;

InitParameter::InitParameter() {
  // Generate object when the first execution
  static bool initial_setup_done = false;
  if (!initial_setup_done) {
    SetSeed();
    InitParameter::uniform_dist_ = new uniform_real_distribution<>(0.0, 1.0);
    InitParameter::normal_dist_ = new normal_distribution<>(0.0, 1.0);
    initial_setup_done = true;
  }

  // No randomization when SetRandomConfig is not called（No setting in MCSim.ini）
  rnd_type_ = NoRandomization;
}

void InitParameter::SetSeed(unsigned long seed, bool is_deterministic) {
  if (is_deterministic) {
    InitParameter::mt_.seed(seed);
  } else {
    InitParameter::mt_.seed(InitParameter::rnd_());
  }
}

void InitParameter::GetDouble(double& dst) const {
  if (rnd_type_ == NoRandomization) {
    ;
  } else if (1 > val_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    dst = val_[0];
  }
}

void InitParameter::GetQuaternion(Quaternion& dst_quat) const {
  if (rnd_type_ == NoRandomization) {
    ;
  } else if (4 > val_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    for (int i = 0; i < 4; i++) {
      dst_quat[i] = val_[i];
    }
  }

  dst_quat.normalize();
}

void InitParameter::Randomize() {
  switch (rnd_type_) {
    case NoRandomization:
      gen_NoRandomization();
      break;
    case CartesianUniform:
      gen_CartesianUniform();
      break;
    case CartesianNormal:
      gen_CartesianNormal();
      break;
    case CircularNormalUniform:
      gen_CircularNormalUniform();
      break;
    case CircularNormalNormal:
      gen_CircularNormalNormal();
      break;
    case SphericalNormalUniformUniform:
      gen_SphericalNormalUniformUniform();
      break;
    case SphericalNormalNormal:
      gen_SphericalNormalNormal();
      break;
    case QuaternionUniform:
      gen_QuaternionUniform();
      break;
    case QuaternionNormal:
      gen_QuaternionNormal();
      break;
    default:
      break;
  }
}

double InitParameter::Uniform_1d(double lb, double ub) { return lb + (*InitParameter::uniform_dist_)(InitParameter::mt_) * (ub - lb); }

double InitParameter::Normal_1d(double mean, double std) { return mean + (*InitParameter::normal_dist_)(InitParameter::mt_) * (std); }

void InitParameter::gen_NoRandomization() { val_.clear(); }

void InitParameter::gen_CartesianUniform() {
  // Random variables following a uniform distribution in Cartesian frame
  val_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    val_.push_back(InitParameter::Uniform_1d(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitParameter::gen_CartesianNormal() {
  // Random variables following a normal distribution in Cartesian frame
  val_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    val_.push_back(InitParameter::Normal_1d(mean_or_min_[i], sigma_or_max_[i]));
  }
}

void InitParameter::get_CircularNormalUniform(Vector<2>& dst, double r_mean, double r_sigma, double theta_min, double theta_max) {
  // r follows normal distribution, and θ follows uniform distribution in Circular frame
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = InitParameter::Uniform_1d(theta_min, theta_max);
  dst[0] = r * cos(theta);
  dst[1] = r * sin(theta);
}

void InitParameter::gen_CircularNormalUniform() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_CircularNormalUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

void InitParameter::get_CircularNormalNormal(Vector<2>& dst, double r_mean, double r_sigma, double theta_mean, double theta_sigma) {
  // r and θ follow normal distribution in Circular frame
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = InitParameter::Normal_1d(theta_mean, theta_sigma);
  dst[0] = r * cos(theta);
  dst[1] = r * sin(theta);
}

void InitParameter::gen_CircularNormalNormal() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_CircularNormalNormal(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

void InitParameter::get_SphericalNormalUniformUniform(Vector<3>& dst, double r_mean, double r_sigma, double theta_min, double theta_max,
                                                      double phi_min, double phi_max) {
  // r follows normal distribution, and θ and φ follow uniform distribution in Spherical frame
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = acos(cos(theta_min) - (cos(theta_min) - cos(theta_max)) * InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(phi_min, phi_max);
  dst[0] = r * sin(theta) * cos(phi);
  dst[1] = r * sin(theta) * sin(phi);
  dst[2] = r * cos(theta);
}

void InitParameter::gen_SphericalNormalUniformUniform() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_SphericalNormalUniformUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0], mean_or_min_[1], sigma_or_max_[1], mean_or_min_[2],
                                    sigma_or_max_[2]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

void InitParameter::get_SphericalNormalNormal(Vector<3>& dst, const Vector<3>& mean_vec) {
  // r and  θ follow normal distribution, and mean vector angle φ follows uniform distribution [0,2*pi]
  Vector<3> mean_vec_dir;
  mean_vec_dir = 1.0 / norm(mean_vec) * mean_vec;  // Unit vector of mean vector direction

  Vector<3> x_axis(0.0), y_axis(0.0);
  x_axis[0] = 1.0;
  y_axis[1] = 1.0;
  Vector<3> op_x = outer_product(mean_vec_dir, x_axis);
  Vector<3> op_y = outer_product(mean_vec_dir, y_axis);

  // An unit vector perpendicular with the mean vector
  // In case of the mean vector is parallel with X or Y axis, selecting the axis depend on the norm of outer product
  Vector<3> normal_unit_vec = norm(op_x) > norm(op_y) ? normalize(op_x) : normalize(op_y);

  double rotation_angle_of_normal_unit_vec = InitParameter::Uniform_1d(0.0, libra::tau);
  Quaternion rotation_of_normal_unit_vec(mean_vec_dir, -rotation_angle_of_normal_unit_vec);  // Use opposite sign to rotate the vector (not the frame)
  Vector<3> rotation_axis = rotation_of_normal_unit_vec.frame_conv(normal_unit_vec);         // Axis of mean vector rotation

  double rotation_angle_of_mean_vec = InitParameter::Normal_1d(0.0, sigma_or_max_[1]);
  Quaternion rotation_of_mean_vec(rotation_axis, -rotation_angle_of_mean_vec);  // Use opposite sign to rotate the vector (not the frame)
  Vector<3> ret_vec = rotation_of_mean_vec.frame_conv(mean_vec_dir);            // Complete calculation of the direction

  ret_vec = InitParameter::Normal_1d(norm(mean_vec), sigma_or_max_[0]) * ret_vec;  // multiply norm

  for (int i = 0; i < 3; i++) {
    dst[i] = ret_vec[i];
  }
}

void InitParameter::gen_SphericalNormalNormal() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < 2) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec, temp_mean_vec;
  for (int i = 0; i < dim; i++) {
    temp_mean_vec[i] = mean_or_min_[i];
  }
  get_SphericalNormalNormal(temp_vec, temp_mean_vec);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

void InitParameter::get_QuaternionUniform(Quaternion& dst) {
  // Perfectly Randomized Quaternion
  Vector<3> x_axis(0.0);
  x_axis[0] = 1.0;

  // A direction vector converted from the X-axis by a quaternion may follows the uniform distribution in full sphere.
  Quaternion first_cnv;
  Vector<3> x_axis_cnvd;
  double theta = acos(1 - (1 - (-1)) * InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(0, libra::tau);
  x_axis_cnvd[0] = sin(theta) * cos(phi);
  x_axis_cnvd[1] = sin(theta) * sin(phi);
  x_axis_cnvd[2] = cos(theta);

  double cos_angle_between = inner_product(x_axis, x_axis_cnvd);
  Vector<3> op = outer_product(x_axis, x_axis_cnvd);
  for (int i = 0; i < 3; i++) {
    first_cnv[i] = op[i];
  }
  first_cnv[3] = cos_angle_between;

  // Generate randomized rotation angle around the X-axis
  double rotation_angle = InitParameter::Uniform_1d(0.0, libra::tau);
  Quaternion second_cnv(x_axis, rotation_angle);

  Quaternion ret_q = first_cnv * second_cnv;

  for (int i = 0; i < 4; i++) {
    dst[i] = ret_q[i];
  }
}

void InitParameter::gen_QuaternionUniform() {
  const static int dim = 4;
  Quaternion temp_q;
  get_QuaternionUniform(temp_q);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_q[i]);
  }
}

void InitParameter::get_QuaternionNormal(Quaternion& dst, double theta_sigma) {
  // Angle from the default quaternion θ follows normal distribution
  // The rotation axis follows uniform distribution on full sphere
  Vector<3> rot_axis;
  double theta = acos(1 - (1 - (-1)) * InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(0, libra::tau);
  rot_axis[0] = sin(theta) * cos(phi);
  rot_axis[1] = sin(theta) * sin(phi);
  rot_axis[2] = cos(theta);

  double rotation_angle = InitParameter::Normal_1d(0.0, theta_sigma);

  Quaternion ret_q(rot_axis, rotation_angle);
  for (int i = 0; i < 4; i++) {
    dst[i] = ret_q[i];
  }
}

void InitParameter::gen_QuaternionNormal() {
  // mean_or_min_[0]: NA   sigma_or_max_[0]: standard deviation of θ [rad]
  const static int dim = 4;
  if (sigma_or_max_.size() < 1) {
    throw "Config parameters dimension unmatched.";
  }
  Quaternion temp_q;
  get_QuaternionNormal(temp_q, sigma_or_max_[0]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_q[i]);
  }
}
