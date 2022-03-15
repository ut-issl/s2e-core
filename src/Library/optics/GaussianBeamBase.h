#pragma once
#include "../math/Vector.hpp"

class GaussianBeamBase {
 public:
  GaussianBeamBase(double wavelength_m, double r_beam_waist_m,
                   double total_power_watt);
  ~GaussianBeamBase();

  // setter
  void SetWaveLength(const double wavelength_m);
  void SetBeamWaistRadius(const double r_beam_waist_m);
  void SetTotalPower(const double total_power_watt);
  void SetPointingVector_i(const libra::Vector<3> pointing_vector_i);
  void SetBeamWaistPos_i(const libra::Vector<3> pos_beamwaist_i);

  // getter
  inline const double GetWaveLength() const { return wavelength_m_; }
  inline const double GetBeamWaistRadius() const { return r_beam_waist_m_; }
  inline const double GetTotalPower() const { return total_power_watt_; }
  inline const libra::Vector<3> GetPointingVector_i() const {
    return pointing_vector_i_;
  }
  inline const libra::Vector<3> GetBeamWaistPos_i() const {
    return pos_beamwaist_i_;
  }

  // Calculate functions
  double CalcBeamWidthRadius(double distance_from_beamwaist_m);
  double CalcIntensity(double distance_from_beamwaist_m,
                       double deviation_from_optical_axis_m);

 private:
  double wavelength_m_;
  double r_beam_waist_m_;
  double total_power_watt_;
  libra::Vector<3> pointing_vector_i_{0.0};
  libra::Vector<3> pos_beamwaist_i_{0.0};
};
