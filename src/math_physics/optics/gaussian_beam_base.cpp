/**
 * @file gaussian_beam_base.cpp
 * @brief Class to express gaussian beam laser
 */

#include "gaussian_beam_base.hpp"

#include <cassert>
#include <math_physics/math/constants.hpp>

namespace optics {

GaussianBeamBase::GaussianBeamBase(double wavelength_m, double radius_beam_waist_m, double total_power_W)
    : wavelength_m_(wavelength_m), radius_beam_waist_m_(radius_beam_waist_m), total_power_W_(total_power_W) {}

GaussianBeamBase::~GaussianBeamBase() {}

void GaussianBeamBase::SetWaveLength_m(const double wavelength_m) {
  assert(wavelength_m > 0.0);
  wavelength_m_ = wavelength_m;
}

void GaussianBeamBase::SetBeamWaistRadius_m(const double radius_beam_waist_m) {
  assert(radius_beam_waist_m > 0.0);
  radius_beam_waist_m_ = radius_beam_waist_m;
}

void GaussianBeamBase::SetTotalPower_W(const double total_power_W) {
  assert(total_power_W >= 0.0);
  total_power_W_ = total_power_W;
}

void GaussianBeamBase::SetPointingVector_i(const math::Vector<3> pointing_vector_i) { pointing_vector_i_ = pointing_vector_i; }

void GaussianBeamBase::SetBeamWaistPosition_i_m(const math::Vector<3> position_beam_waist_i_m) { position_beam_waist_i_m_ = position_beam_waist_i_m; }

double GaussianBeamBase::CalcBeamWidthRadius_m(double distance_from_beam_waist_m) {
  double rayleigh_length_m = math::pi * radius_beam_waist_m_ * radius_beam_waist_m_ / wavelength_m_;
  double beam_width_radius_m = radius_beam_waist_m_ * sqrt(1.0 + std::pow((distance_from_beam_waist_m / rayleigh_length_m), 2.0));
  return beam_width_radius_m;
}

double GaussianBeamBase::CalcIntensity_W_m2(double distance_from_beam_waist_m, double deviation_from_optical_axis_m) {
  double beam_width_radius_m = CalcBeamWidthRadius_m(distance_from_beam_waist_m);
  double peak_intensity_W_m2 = (2.0 * total_power_W_) / (math::pi * beam_width_radius_m * beam_width_radius_m);
  double gaussian_dist =
      std::exp((-2.0 * deviation_from_optical_axis_m * deviation_from_optical_axis_m) / (beam_width_radius_m * beam_width_radius_m));
  double intensity_W_m2 = peak_intensity_W_m2 * gaussian_dist;
  return intensity_W_m2;
}

}  // namespace optics
