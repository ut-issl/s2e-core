/**
 * @file vector.cpp
 * @brief Class for mathematical vector
 */

#include "vector.hpp"

#include "constants.hpp"

namespace math {
Vector<3, double> ConvertFrameOrthogonal2Polar(const Vector<3, double>& orthogonal) {
  Vector<3, double> polar;  // vector on the polar coordinate
  polar.FillUp(0.0);
  polar[0] = orthogonal.CalcNorm();
  // Skip when zero vector
  if (polar[0] == 0.0) {
    return polar;
  }
  polar[1] = acos(orthogonal[2] / polar[0]);
  // Skip phi calculation when the orthogonal is on the Z-axis
  if ((orthogonal[0] == 0.0) && (orthogonal[1] == 0.0)) {
    return polar;
  }
  polar[2] = atan2(orthogonal[1], orthogonal[0]);
  if (polar[2] < 0.0) {
    polar[2] += numbers::tau;
  }

  return polar;
}

Vector<3, double> GenerateOrthogonalUnitVector(const Vector<3, double>& v) {
  Vector<3> orthogonal_vector;
  if (v[0] * v[0] <= v[1] * v[1] && v[0] * v[0] <= v[1] * v[1]) {
    orthogonal_vector[0] = 0.0;
    orthogonal_vector[1] = v[2];
    orthogonal_vector[2] = -v[1];
    orthogonal_vector = orthogonal_vector.CalcNormalizedVector();
    return (orthogonal_vector);
  } else if (v[1] * v[1] <= v[0] * v[0] && v[1] * v[1] <= v[2] * v[2]) {
    orthogonal_vector[0] = -v[2];
    orthogonal_vector[1] = 0.0;
    orthogonal_vector[2] = v[0];
    orthogonal_vector = orthogonal_vector.CalcNormalizedVector();
    return (orthogonal_vector);
  } else {
    orthogonal_vector[0] = v[1];
    orthogonal_vector[1] = -v[0];
    orthogonal_vector[2] = 0.0;
    orthogonal_vector = orthogonal_vector.CalcNormalizedVector();
    return (orthogonal_vector);
  }
}
}  // namespace libra
