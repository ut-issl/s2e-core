/**
 * @file gaussian_beam_base.cpp
 * @brief Class to express gaussian beam laser
 */

#include "gaussian_beam_base.hpp"

#include <cassert>
#include <library/math/constants.hpp>

GaussianBeamBase::GaussianBeamBase(double wavelength_m, double r_beam_waist_m, double total_power_watt)
    : wavelength_m_(wavelength_m), radius_beam_waist_m_(r_beam_waist_m), total_power_W_(total_power_watt) {}

GaussianBeamBase::~GaussianBeamBase() {}

void GaussianBeamBase::SetWaveLength(const double wavelength_m) {
  assert(wavelength_m > 0.0);
  wavelength_m_ = wavelength_m;
}

void GaussianBeamBase::SetBeamWaistRadius(const double r_beam_waist_m) {
  assert(r_beam_waist_m > 0.0);
  radius_beam_waist_m_ = r_beam_waist_m;
}

void GaussianBeamBase::SetTotalPower(const double total_power_watt) {
  assert(total_power_watt >= 0.0);
  total_power_W_ = total_power_watt;
}

void GaussianBeamBase::SetPointingVector_i(const libra::Vector<3> pointing_vector_i) { pointing_vector_i_ = pointing_vector_i; }

void GaussianBeamBase::SetBeamWaistPos_i(const libra::Vector<3> pos_beamwaist_i) { position_beam_waist_i_m_ = pos_beamwaist_i; }

double GaussianBeamBase::CalcBeamWidthRadius(double distance_from_beamwaist_m) {
  double rayleigh_length_m = libra::pi * radius_beam_waist_m_ * radius_beam_waist_m_ / wavelength_m_;
  double beam_width_radius_m = radius_beam_waist_m_ * sqrt(1.0 + std::pow((distance_from_beamwaist_m / rayleigh_length_m), 2.0));
  return beam_width_radius_m;
}

double GaussianBeamBase::CalcIntensity(double distance_from_beamwaist_m, double deviation_from_optical_axis_m) {
  double beam_width_radius_m = CalcBeamWidthRadius(distance_from_beamwaist_m);
  double peak_intensity_watt_m2 = (2.0 * total_power_W_) / (libra::pi * beam_width_radius_m * beam_width_radius_m);
  double gaussian_dist =
      std::exp((-2.0 * deviation_from_optical_axis_m * deviation_from_optical_axis_m) / (beam_width_radius_m * beam_width_radius_m));
  double intensity_watt_m2 = peak_intensity_watt_m2 * gaussian_dist;
  return intensity_watt_m2;
}
