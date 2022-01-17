#include "InitParameter.h"

using namespace std;

random_device InitParameter::rnd_;
mt19937 InitParameter::mt_;
uniform_real_distribution<> *InitParameter::uniform_dist_;
normal_distribution<> *InitParameter::normal_dist_;

InitParameter::InitParameter() {
  //初回実行時のみオブジェクトを生成する
  static bool initial_setup_done = false;
  if (!initial_setup_done) {
    SetSeed();
    InitParameter::uniform_dist_ = new uniform_real_distribution<>(0.0, 1.0);
    InitParameter::normal_dist_ = new normal_distribution<>(0.0, 1.0);
    initial_setup_done = true;
  }

  // SetRandomConfigが呼ばれない場合（MCSim.iniに設定されていない場合）はRandomizeしない
  rnd_type_ = NoRandomization;
}

void InitParameter::SetSeed(unsigned long seed, bool is_deterministic) {
  if (is_deterministic) {
    InitParameter::mt_.seed(seed);
  } else {
    InitParameter::mt_.seed(InitParameter::rnd_());
  }
}

void InitParameter::GetDouble(double &dst) const {
  if (rnd_type_ == NoRandomization) {
    ;
  } else if (1 > val_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    dst = val_[0];
  }
}

void InitParameter::GetQuaternion(Quaternion &dst_quat) const {
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
  case NoRandomization: // 乱数生成なし（デフォルト値を出力）
    gen_NoRandomization();
    break;
  case CartesianUniform: // 直交座標系での各成分が一様分布に従う確率変数
    gen_CartesianUniform();
    break;
  case CartesianNormal: // 直交座標系での各成分が正規分布に従う確率変数
    gen_CartesianNormal();
    break;
  case CircularNormalUniform: // 円座標系においてrが正規分布，θが一様分布に従う
    gen_CircularNormalUniform();
    break;
  case CircularNormalNormal: // 円座標系においてrとθが正規分布に従う
    gen_CircularNormalNormal();
    break;
  case SphericalNormalUniformUniform: // 球座標系においてrが正規分布，θ・φが一様分布に従う
    gen_SphericalNormalUniformUniform();
    break;
  case SphericalNormalNormal: // ベクトルのノルム(r)とデフォルトベクトルとの角度(θ)が正規分布に従う．デフォルトベクトル周りの角度(φ)は[0,2*pi]の一様分布．
    gen_SphericalNormalNormal();
    break;
  case QuaternionUniform: // 完全にランダムな姿勢を出力するクオータニオン確率変数．
    gen_QuaternionUniform();
    break;
  case QuaternionNormal: // デフォルトクオータニオンからの角度のずれ(θ)が正規分布に従う．
    gen_QuaternionNormal();
    break;
  default:
    break;
  }
}

double InitParameter::Uniform_1d(double lb, double ub) {
  return lb + (*InitParameter::uniform_dist_)(InitParameter::mt_) * (ub - lb);
}

double InitParameter::Normal_1d(double mean, double std) {
  return mean + (*InitParameter::normal_dist_)(InitParameter::mt_) * (std);
}

// 乱数生成なし（GetVec, GetDoubleで与えられる参照の変数を変更しない）
void InitParameter::gen_NoRandomization() { val_.clear(); }

// 直交座標系での各成分が一様分布に従う確率変数（現状3次元まで）
// mean_or_min_[0]: x成分最小値   sigma_or_max_[0]: x成分最大値
// mean_or_min_[1]: y成分最小値   sigma_or_max_[1]: y成分最大値
// mean_or_min_[2]: z成分最小値   sigma_or_max_[2]: z成分最大値
void InitParameter::gen_CartesianUniform() {
  val_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    val_.push_back(
        InitParameter::Uniform_1d(mean_or_min_[i], sigma_or_max_[i]));
  }
}

// 直交座標系での各成分が正規分布に従う確率変数（現状3次元まで）
// mean_or_min_[0]: x成分平均   sigma_or_max_[0]: x成分標準偏差
// mean_or_min_[1]: y成分平均   sigma_or_max_[1]: y成分標準偏差
// mean_or_min_[2]: z成分平均   sigma_or_max_[2]: z成分標準偏差
void InitParameter::gen_CartesianNormal() {
  val_.clear();
  for (unsigned int i = 0; i < mean_or_min_.size(); i++) {
    val_.push_back(InitParameter::Normal_1d(mean_or_min_[i], sigma_or_max_[i]));
  }
}

// 円座標系においてrが正規分布，θが一様分布に従う
void InitParameter::get_CircularNormalUniform(Vector<2> &dst, double r_mean,
                                              double r_sigma, double theta_min,
                                              double theta_max) {
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = InitParameter::Uniform_1d(theta_min, theta_max);
  dst[0] = r * cos(theta);
  dst[1] = r * sin(theta);
}

// mean_or_min_[0]: r成分平均     sigma_or_max_[0]: r成分標準偏差
// mean_or_min_[1]: θ成分最小値   sigma_or_max_[1]: θ成分最大値
void InitParameter::gen_CircularNormalUniform() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_CircularNormalUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0],
                            mean_or_min_[1], sigma_or_max_[1]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

// 円座標系においてrとθが正規分布に従う
void InitParameter::get_CircularNormalNormal(Vector<2> &dst, double r_mean,
                                             double r_sigma, double theta_mean,
                                             double theta_sigma) {
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = InitParameter::Normal_1d(theta_mean, theta_sigma);
  dst[0] = r * cos(theta);
  dst[1] = r * sin(theta);
}

// mean_or_min_[0]: r成分平均   sigma_or_max_[0]: r成分標準偏差
// mean_or_min_[1]: θ成分平均   sigma_or_max_[1]: θ成分標準偏差
void InitParameter::gen_CircularNormalNormal() {
  const static int dim = 2;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_CircularNormalNormal(temp_vec, mean_or_min_[0], sigma_or_max_[0],
                           mean_or_min_[1], sigma_or_max_[1]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

// 球座標系においてrが正規分布，θ・φが球面一様分布に従う
// θ・φは球面に対して一様な分布をするように計算されている．
void InitParameter::get_SphericalNormalUniformUniform(
    Vector<3> &dst, double r_mean, double r_sigma, double theta_min,
    double theta_max, double phi_min, double phi_max) {
  double r = InitParameter::Normal_1d(r_mean, r_sigma);
  double theta = acos(cos(theta_min) - (cos(theta_min) - cos(theta_max)) *
                                           InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(phi_min, phi_max);
  dst[0] = r * sin(theta) * cos(phi);
  dst[1] = r * sin(theta) * sin(phi);
  dst[2] = r * cos(theta);
}

// mean_or_min_[0]: r成分平均     sigma_or_max_[0]: r成分標準偏差
// mean_or_min_[1]: θ成分最小値   sigma_or_max_[1]: θ成分最大値
// mean_or_min_[2]: φ成分最小値   sigma_or_max_[2]: φ成分最大値
void InitParameter::gen_SphericalNormalUniformUniform() {
  const static int dim = 3;
  if (mean_or_min_.size() < dim || sigma_or_max_.size() < dim) {
    throw "Config parameters dimension unmatched.";
  }
  Vector<dim> temp_vec;
  get_SphericalNormalUniformUniform(temp_vec, mean_or_min_[0], sigma_or_max_[0],
                                    mean_or_min_[1], sigma_or_max_[1],
                                    mean_or_min_[2], sigma_or_max_[2]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

// ベクトルのノルム(r)とmean vectorとの角度(θ)が正規分布に従う．
// mean vector周りの角度(φ)は[0,2*pi]の一様分布．
// mean_or_min_[0]: x成分平均    sigma_or_max_[0]: r成分標準偏差
// mean_or_min_[1]: y成分平均    sigma_or_max_[1]: θ成分標準偏差
// mean_or_min_[2]: z成分平均    sigma_or_max_[2]: なし
void InitParameter::get_SphericalNormalNormal(Vector<3> &dst,
                                              const Vector<3> &mean_vec,
                                              double r_sigma,
                                              double theta_sigma) {
  Vector<3> mean_vec_dir;
  mean_vec_dir =
      1.0 / norm(mean_vec) * mean_vec; // mean vector方向の単位ベクトル

  Vector<3> x_axis(0.0), y_axis(0.0);
  x_axis[0] = 1.0;
  y_axis[1] = 1.0;
  Vector<3> op_x = outer_product(mean_vec_dir, x_axis);
  Vector<3> op_y = outer_product(mean_vec_dir, y_axis);

  // mean vectorに垂直な単位ベクトルの一つ．mean
  // vectorがx軸またはy軸と平行だった場合に備えて，外積ベクトルノルムの大きい方との外積を選ぶ．
  Vector<3> normal_unit_vec =
      norm(op_x) > norm(op_y) ? normalize(op_x) : normalize(op_y);

  double rotation_angle_of_normal_unit_vec =
      InitParameter::Uniform_1d(0.0, 2 * M_PI);
  Quaternion rotation_of_normal_unit_vec(
      mean_vec_dir,
      -rotation_angle_of_normal_unit_vec); //座標が回転するのではなくベクトルが回転するので角度の向きを逆にする
  Vector<3> rotation_axis = rotation_of_normal_unit_vec.frame_conv(
      normal_unit_vec); // mean vectorを回転させる軸

  double rotation_angle_of_mean_vec =
      InitParameter::Normal_1d(0.0, sigma_or_max_[1]);
  Quaternion rotation_of_mean_vec(
      rotation_axis,
      -rotation_angle_of_mean_vec); //座標が回転するのではなくベクトルが回転するので角度の向きを逆にする
  Vector<3> ret_vec =
      rotation_of_mean_vec.frame_conv(mean_vec_dir); //方向の計算完了

  ret_vec = InitParameter::Normal_1d(norm(mean_vec), sigma_or_max_[0]) *
            ret_vec; //ノルム掛け算

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
  get_SphericalNormalNormal(temp_vec, temp_mean_vec, sigma_or_max_[0],
                            sigma_or_max_[1]);

  val_.clear();
  for (int i = 0; i < dim; i++) {
    val_.push_back(temp_vec[i]);
  }
}

// 完全にランダムな姿勢を出力するクオータニオン確率変数．
void InitParameter::get_QuaternionUniform(Quaternion &dst) {
  Vector<3> x_axis(0.0);
  x_axis[0] = 1.0;

  // x軸がクオータニオンによって変換された後の方向ベクトルは球面上に一様な確率分布を持つはずである．
  Quaternion first_cnv;
  Vector<3> x_axis_cnvd;
  double theta = acos(1 - (1 - (-1)) * InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(0, 2 * M_PI);
  x_axis_cnvd[0] = sin(theta) * cos(phi);
  x_axis_cnvd[1] = sin(theta) * sin(phi);
  x_axis_cnvd[2] = cos(theta);

  double cos_angle_between = inner_product(x_axis, x_axis_cnvd);
  Vector<3> op = outer_product(x_axis, x_axis_cnvd);
  for (int i = 0; i < 3; i++) {
    first_cnv[i] = op[i];
  }
  first_cnv[3] = cos_angle_between;

  // x軸周りの回転角をランダムに生成する
  double rotation_angle = InitParameter::Uniform_1d(0.0, 2 * M_PI);
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

// デフォルトクオータニオンからの角度のずれ(θ)が正規分布に従う．
void InitParameter::get_QuaternionNormal(Quaternion &dst, double theta_sigma) {
  // 回転軸は全球面上に一様分布
  Vector<3> rot_axis;
  double theta = acos(1 - (1 - (-1)) * InitParameter::Uniform_1d(0.0, 1.0));
  double phi = InitParameter::Uniform_1d(0, 2 * M_PI);
  rot_axis[0] = sin(theta) * cos(phi);
  rot_axis[1] = sin(theta) * sin(phi);
  rot_axis[2] = cos(theta);

  double rotation_angle = InitParameter::Normal_1d(0.0, theta_sigma);

  Quaternion ret_q(rot_axis, rotation_angle);
  for (int i = 0; i < 4; i++) {
    dst[i] = ret_q[i];
  }
}

// mean_or_min_[0]: なし   sigma_or_max_[0]: θ標準偏差
void InitParameter::gen_QuaternionNormal() {
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
