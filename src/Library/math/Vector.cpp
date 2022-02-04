/*!
  \file   Vector.cpp
  \author TAKISAWA Jun'ichi.
  \date   Sat Nov  6 02:44:24 2010
  
  \brief  Vector.hppで宣言された非template関数の定義
*/
#include "Vector.hpp"

#include <cmath>

namespace libra
{
  Vector<3, double> ortho2spher(const Vector<3, double>& ortho)
  {
    // 球座標結果格納先。全て0.0で初期化する。
    Vector<3, double> spher; fill_up(spher, 0.0);
    spher[0] = norm(ortho);
    // 零ベクトルの場合角度計算をスキップする。
    if(spher[0] == 0.0){ return spher; }
    spher[1] = acos(ortho[2]/spher[0]);
    // ベクトルがz軸上の場合phi計算をスキップする。
    if((ortho[0] == 0.0) && (ortho[1] == 0.0)){ return spher; }
    spher[2] = atan2(ortho[1], ortho[0]);
    if(spher[2] < 0.0){ spher[2]+=2.0*M_PI; }
    
    return spher;
  }

  Vector<3, double> ortho2lonlat(const Vector<3, double>& ortho)
  {
    Vector<3, double> lonlat; fill_up(lonlat, 0.0);
    lonlat[0] = norm(ortho);
    // 零ベクトルの場合角度計算をスキップする。
    if(lonlat[0] == 0.0){ return lonlat; }
    lonlat[1] = 0.5*M_PI - acos(ortho[2]/lonlat[0]);
    // ベクトルがz軸上の場合phi計算をスキップする。
    if((ortho[0] == 0.0) && (ortho[1] == 0.0)){ return lonlat; }
    lonlat[2] = atan2(ortho[1], ortho[0]);

    return lonlat;
  }

  Vector<3, double> GenerateOrthoUnitVector(const Vector<3, double>& v)
  {
    Vector<3> v_ortho;
    if (v[0] * v[0] <= v[1] * v[1] && v[0] * v[0] <= v[1] * v[1])
    {
      v_ortho[0] = 0.0;
      v_ortho[1] = v[2];
      v_ortho[2] = -v[1];
      v_ortho = normalize(v_ortho);
      return(v_ortho);
    }
    else if (v[1] * v[1] <= v[0] * v[0] && v[1] * v[1] <= v[2] * v[2])
    {
      v_ortho[0] = -v[2];
      v_ortho[1] = 0.0;
      v_ortho[2] = v[0];
      v_ortho = normalize(v_ortho);
      return(v_ortho);
    }
    else
    {
      v_ortho[0] = v[1];
      v_ortho[1] = -v[0];
      v_ortho[2] = 0.0;
      v_ortho = normalize(v_ortho);
      return(v_ortho);
    }
  }
} // libra
