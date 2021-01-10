#ifndef __orbit_H__
#define __orbit_H__

#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"

using libra::Matrix;
using libra::Vector;
using libra::Quaternion;

#include "../../Interface/LogOutput/ILoggable.h"

// For ECI to GEO calculation
#include <math.h>
#include "../../Library/sgp4/sgp4unit.h"
#include "../../Library/sgp4/sgp4io.h"
#include "../../Library/sgp4/sgp4ext.h"

#define PIO2		1.57079632679489656		/* Pi/2 */
#define TWOPI		6.28318530717958623		/* 2*Pi  */
#define DEG2RAD		0.017453292519943295769	// PI/180

class Orbit: public ILoggable
{
public:
  virtual ~Orbit() {}

  inline Vector<3> GetSatPosition_i() const { return sat_position_i_; }
  inline Vector<3> GetSatPosition_ecef() const { return sat_position_ecef_; }
  inline Vector<3> GetSatVelocity_i() const { return sat_velocity_i_; }
  inline Vector<3> GetSatVelocity_b() const { return sat_velocity_b_; }

  inline double GetLat_rad() const { return lat_rad_; }
  inline double GetLon_rad() const { return lon_rad_; }
  inline double GetAlt_m() const { return alt_m_; }
  inline Vector<3> GetLatLonAlt() const
  {
    Vector<3> vec;
    vec(0) = lat_rad_; vec(1) = lon_rad_; vec(2) = alt_m_;
    return vec;
  }

  // 初期化関数（実装は任意）
  inline virtual void Initialize(Vector<3> init_position, Vector<3> init_velocity, double current_jd, double init_time = 0) {}

  //軌道のプロパゲーション
  virtual void Propagate(double endtime, double current_jd) = 0;

  // 姿勢情報の更新
  inline void UpdateAtt(Quaternion q_i2b) { sat_velocity_b_ = q_i2b.frame_conv(sat_velocity_i_); }

  inline void SetAcceleration_i(Vector<3> acceleration_i)
  {
    acc_i_ = acceleration_i;
  }

  // 慣性系での並進力をセットする
  inline void AddForce_i(Vector<3> force_i, double spacecraft_mass)
  {
    force_i /= spacecraft_mass;
    acc_i_ += force_i;
  }

  // 慣性系での加速度をセットする
  inline void AddAcceleration_i(Vector<3> acceleration_i)
  {
    acc_i_ += acceleration_i;
  }

  // 機体座標系での並進力をセットする
  inline void AddForce_b(Vector<3> force_b, Quaternion q_i2b, double spacecraft_mass)
  {
    auto force_i = q_i2b.frame_conv_inv(force_b);
    AddForce_i(force_i, spacecraft_mass);
  }

  // 宇宙機の位置を指定した慣性座標系でのオフセットベクトルだけずらす
  inline virtual void AddPositionOffset(Vector<3> offset_i) {}

  // Convert ECI satellite position to ECEF frame
  inline void TransECIToECEF(double current_jd)
  {
    double current_side = gstime(current_jd);

    Matrix<3, 3> trans_mat;  // グリニッジ恒星時により地球の自転を表したECI2ECEF変換行列
    trans_mat[0][0] = cos(current_side);
    trans_mat[0][1] = sin(current_side);
    trans_mat[0][2] = 0;
    trans_mat[1][0] = -sin(current_side);
    trans_mat[1][1] = cos(current_side);
    trans_mat[1][2] = 0;
    trans_mat[2][0] = 0;
    trans_mat[2][1] = 0;
    trans_mat[2][2] = 1;

    sat_position_ecef_ = trans_mat * sat_position_i_;
    trans_eci2ecef_ = trans_mat;
  }

  // Convert ECI satellite position to GEO(lattitude, longitude, and latitude)
  void TransECIToGeo(double current_jd)
  {
    double r, e2, phi, c;
    double theta;
    double current_side = gstime(current_jd);
    double radiusearthkm;
    double f;
    getwgsconst(whichconst, radiusearthkm, f);

    theta = AcTan(sat_position_i_[1], sat_position_i_[0]); /* radians */
    lon_rad_ = FMod2p(theta - current_side); /* radians */
    r = sqrt(sat_position_i_[0] * sat_position_i_[0] + sat_position_i_[1] * sat_position_i_[1]);
    e2 = f * (2.0 - f);
    lat_rad_ = AcTan(sat_position_i_[2], r); /* radians */

    do
    {
      phi = lat_rad_;
      c = 1.0 / sqrt(1.0 - e2 * sin(phi)*sin(phi));
      lat_rad_ = AcTan(sat_position_i_[2] + radiusearthkm * c*e2*sin(phi) * 1000.0, r);

    } while (fabs(lat_rad_ - phi) >= 1E-10);

    alt_m_ = r / cos(lat_rad_) - radiusearthkm * c * 1000.0; /* kilometers -> meters */	//楕円体高さ

    if (lat_rad_ > PIO2)
      lat_rad_ -= TWOPI;
  }

  virtual string GetLogHeader() const = 0;
  virtual string GetLogValue() const = 0;

  inline virtual Vector<3> GetESIOmega()const { return Vector<3>();}

  inline Matrix<3,3> GetTransECItoECEF()const { return trans_eci2ecef_; }

  bool IsCalcEnabled = false;
  gravconsttype whichconst;

protected:
  // 慣性系での宇宙機位置 [m]
  Vector<3> sat_position_i_;

  // ECEFでの宇宙機位置 [m]
  Vector<3> sat_position_ecef_;

  // 慣性系での宇宙機速度 [m/s]
  Vector<3> sat_velocity_i_;

  // 機体座標系での宇宙機速度 [m/s]
  Vector<3> sat_velocity_b_;

  // 慣性系での宇宙機加速度 [m/s2]
  // Propagate関数の末尾でゼロクリアしてください
  Vector<3> acc_i_;

  //緯度[rad]南緯：-π/2〜0,北緯：0〜π/2
  double lat_rad_;

  //経度[rad]東経：0〜π,西経:2π〜π（すなわちグリニッジ子午線から東に0〜２π[rad]と定義）
  double lon_rad_;

  //高度[m]
  double alt_m_;

  //TransECItoECEF
  Matrix<3,3> trans_eci2ecef_;
};

#endif //__orbit_H__
