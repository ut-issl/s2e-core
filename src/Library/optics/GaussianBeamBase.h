#pragma once
#include "../math/Vector.hpp"

class GaussianBeamBase
{
public:
  GaussianBeamBase(double wavelength_m, double r_beam_waist_m, double total_power_watt);
  ~GaussianBeamBase();
  void Update(libra::Vector<3> pointing_vector_i, libra::Vector<3> pos_beamwaist_i);

  //setter
  void SetWaveLength(const double wavelength_m);
  void SetBeamWaistRadius(const double r_beam_waist_m);
  void SetTotalPower(const double total_power_watt);

  //getter
  inline const double GetWaveLength() const { return wavelength_m_; }
  inline const double GetBeamWaistRadius() const { return r_beam_waist_m_; }
  inline const double GetTotalPower() const { return total_power_watt_; }
  inline libra::Vector<3> GetPointingVector_i() { return  pointing_vector_i_; };
  inline libra::Vector<3> GetBeamWaistPos_i() { return pos_beamwaist_i_; };

private:
  double wavelength_m_;
  double r_beam_waist_m_;
  double total_power_watt_;
  libra::Vector<3> pointing_vector_i_{ 0.0 };
  libra::Vector<3> pos_beamwaist_i_{ 0.0 };
};
