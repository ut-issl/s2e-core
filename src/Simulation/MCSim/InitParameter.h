#pragma once

#include <random>
#include <string>
#include <vector>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include <math.h>

using libra::Quaternion;
using libra::Vector;

class InitParameter {
public:
  // 乱数タイプ
  enum RandomizationType {
    NoRandomization, // 乱数生成なし（デフォルト値を出力）
    CartesianUniform, // 直交座標系での各成分が一様分布に従う確率変数
    CartesianNormal, // 直交座標系での各成分が正規分布に従う確率変数
    CircularNormalUniform, // 円座標系においてrが正規分布，θが一様分布に従う
    CircularNormalNormal, // 円座標系においてrとθが正規分布に従う
    SphericalNormalUniformUniform, // 球座標系においてrが正規分布，θ・φが球面一様分布に従う
    SphericalNormalNormal, // ベクトルのノルム(r)とデフォルトベクトルとの角度(θ)が正規分布に従う．mean
                           // vector周りの角度(φ)は[0,2*pi]の一様分布．
    QuaternionUniform, // 完全にランダムな姿勢を出力するクオータニオン確率変数．
    QuaternionNormal, // デフォルトクオータニオンからの角度のずれ(θ)が正規分布に従う．
  };

  // コンストラクタ
  InitParameter();

  //// コピーコンストラクタ
  // InitParameter(const InitParameter& other);

  //// デストラクタ
  //~InitParameter();

  // Randomization seedの設定．is_deterministic =
  // falaseの場合は引数seedは用いられず時刻情報が用いられる．
  static void SetSeed(unsigned long seed = 0, bool is_deterministic = false);

  template <size_t NumElement1, size_t NumElement2>
  // Ramdomizationのパラメータを設定する
  void SetRandomConfig(const Vector<NumElement1> &mean_or_min,
                       const Vector<NumElement2> &sigma_or_max,
                       RandomizationType rnd_type);

  // Randomizationのパラメータを用いて実際にRanzomizeする
  void Randomize();

  template <size_t NumElement>
  // Ranzomizeした結果の値をdst_vecに格納する
  void GetVec(Vector<NumElement> &dst_vec) const;

  // Ranzomizeした結果の値をdst_quatに格納する
  void GetQuaternion(Quaternion &dst_quat) const;

  // Ranzomizeした結果の値をdstに格納する
  void GetDouble(double &dst) const;

private:
  // 実際の値
  std::vector<double> val_;

  // 平均値または最小値（使われ方はgen_[RandomizationType]関数のコメントを参照）
  std::vector<double> mean_or_min_;

  // 標準偏差または最大値（使われ方はgen_[RandomizationType]関数のコメントを参照）
  std::vector<double> sigma_or_max_;

  // For randomization
  RandomizationType rnd_type_; // 乱数タイプ
  static std::random_device
      rnd_; // 非決定的乱数生成器（時刻に基づいて乱数を生成し，非決定的なseedを生成する）
  static std::mt19937 mt_; // 決定的乱数生成器
  static std::uniform_real_distribution<> *uniform_dist_; // 一様分布乱数生成器
  static std::normal_distribution<> *normal_dist_; // 正規分布乱数生成器

  // 一様分布する1次元乱数を生成する
  static double Uniform_1d(double lb, double ub);

  // 正規分布する1次元乱数を生成する
  static double Normal_1d(double mean, double std);

  void gen_NoRandomization();
  void gen_CartesianUniform();
  void gen_CartesianNormal();
  void gen_CircularNormalUniform();
  void gen_CircularNormalNormal();
  void gen_SphericalNormalUniformUniform();
  void gen_SphericalNormalNormal();
  void gen_QuaternionUniform();
  void gen_QuaternionNormal();

  void get_CircularNormalUniform(Vector<2> &dst, double r_mean, double r_sigma,
                                 double theta_min, double theta_max);
  void get_CircularNormalNormal(Vector<2> &dst, double r_mean, double r_sigma,
                                double theta_mean, double theta_sigma);
  void get_SphericalNormalUniformUniform(Vector<3> &dst, double r_mean,
                                         double r_sigma, double theta_min,
                                         double theta_max, double phi_min,
                                         double phi_max);
  void get_SphericalNormalNormal(Vector<3> &dst, const Vector<3> &mean_vec,
                                 double r_sigma, double theta_sigma);
  void get_QuaternionUniform(Quaternion &dst);
  void get_QuaternionNormal(Quaternion &dst, double theta_sigma);
};

template <size_t NumElement1, size_t NumElement2>
void InitParameter::SetRandomConfig(const Vector<NumElement1> &mean_or_min,
                                    const Vector<NumElement2> &sigma_or_max,
                                    InitParameter::RandomizationType rnd_type) {
  rnd_type_ = rnd_type;
  mean_or_min_.clear();
  for (int i = 0; i < NumElement1; i++) {
    mean_or_min_.push_back(mean_or_min[i]);
  }
  sigma_or_max_.clear();
  for (int i = 0; i < NumElement2; i++) {
    sigma_or_max_.push_back(sigma_or_max[i]);
  }
}

template <size_t NumElement>
void InitParameter::GetVec(Vector<NumElement> &dst_vec) const {
  if (rnd_type_ == NoRandomization) {
    ;
  } else if (NumElement > val_.size()) {
    throw "Too few randomization configuration parameters.";
  } else {
    for (int i = 0; i < NumElement; i++) {
      dst_vec[i] = val_[i];
    }
  }
}
