/**
 * @file quantization.cpp
 * @brief Functions for quantization
 */

#include "quantization.hpp"

namespace s2e::utilities {

double quantization(const double continuous_number, const double resolution) {
  int bin_num = (int)((double)continuous_number / resolution);
  return (double)bin_num * resolution;
}

float quantization_float(const double continuous_number, const double resolution) { return (float)quantization(continuous_number, resolution); }

}  // namespace s2e::utilities
