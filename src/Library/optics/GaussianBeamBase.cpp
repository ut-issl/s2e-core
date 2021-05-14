#define _USE_MATH_DEFINES
#include "GaussianBeamBase.h"
#include <cassert>
#include <cmath>

GaussianBeamBase::GaussianBeamBase(double wavelength_m, double r_beam_waist_m, double total_power_watt) 
  : wavelength_m_(wavelength_m), 
   r_beam_waist_m_(r_beam_waist_m), 
   total_power_watt_(total_power_watt)
{
}

GaussianBeamBase::~GaussianBeamBase()
{
}

void GaussianBeamBase::Update(libra::Vector<3> pointing_vector_i, libra::Vector<3> pos_beamwaist_i)
{
  pointing_vector_i_ = pointing_vector_i;
  pos_beamwaist_i_ = pos_beamwaist_i;
}

void GaussianBeamBase::SetWaveLength(const double wavelength_m)
{
  assert(wavelength_m > 0.0);
  wavelength_m_ = wavelength_m;
}

void GaussianBeamBase::SetBeamWaistRadius(const double r_beam_waist_m)
{
  assert(r_beam_waist_m > 0.0);
  r_beam_waist_m_ = r_beam_waist_m;
}

void GaussianBeamBase::SetTotalPower(const double total_power_watt)
{
  assert(total_power_watt >= 0.0);
  total_power_watt_ = total_power_watt;
}
