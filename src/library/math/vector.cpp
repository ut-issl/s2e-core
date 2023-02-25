/**
 * @file vector.cpp
 * @brief Class for mathematical vector
 */

#include "vector.hpp"

#include "constants.hpp"

namespace libra {
Vector<3, double> ConvertFrameOrthogonal2Polar(const Vector<3, double>& ortho) {
  Vector<3, double> spher;  // vector on the polar coordinate
  FillUp(spher, 0.0);
  spher[0] = CalcNorm(ortho);
  // Skip when zero vector
  if (spher[0] == 0.0) {
    return spher;
  }
  spher[1] = acos(ortho[2] / spher[0]);
  // Skip phi calculation when the ortho is on the Z-axis
  if ((ortho[0] == 0.0) && (ortho[1] == 0.0)) {
    return spher;
  }
  spher[2] = atan2(ortho[1], ortho[0]);
  if (spher[2] < 0.0) {
    spher[2] += numbers::tau;
  }

  return spher;
}

Vector<3, double> ortho2lonlat(const Vector<3, double>& ortho) {
  Vector<3, double> lonlat;
  FillUp(lonlat, 0.0);
  lonlat[0] = CalcNorm(ortho);
  // Skip when zero vector
  if (lonlat[0] == 0.0) {
    return lonlat;
  }
  lonlat[1] = numbers::pi_2 - acos(ortho[2] / lonlat[0]);
  // Skip phi calculation when the ortho is on the Z-axis
  if ((ortho[0] == 0.0) && (ortho[1] == 0.0)) {
    return lonlat;
  }
  lonlat[2] = atan2(ortho[1], ortho[0]);

  return lonlat;
}

Vector<3, double> GenerateOrthogonalUnitVector(const Vector<3, double>& v) {
  Vector<3> v_ortho;
  if (v[0] * v[0] <= v[1] * v[1] && v[0] * v[0] <= v[1] * v[1]) {
    v_ortho[0] = 0.0;
    v_ortho[1] = v[2];
    v_ortho[2] = -v[1];
    v_ortho = Normalize(v_ortho);
    return (v_ortho);
  } else if (v[1] * v[1] <= v[0] * v[0] && v[1] * v[1] <= v[2] * v[2]) {
    v_ortho[0] = -v[2];
    v_ortho[1] = 0.0;
    v_ortho[2] = v[0];
    v_ortho = Normalize(v_ortho);
    return (v_ortho);
  } else {
    v_ortho[0] = v[1];
    v_ortho[1] = -v[0];
    v_ortho[2] = 0.0;
    v_ortho = Normalize(v_ortho);
    return (v_ortho);
  }
}
}  // namespace libra
