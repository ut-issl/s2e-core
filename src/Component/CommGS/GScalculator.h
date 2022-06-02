/*
 * @file GScalculator.h
 * @brief Emuration of analysis and calculation for Ground Stations
 * @author 山本 智貴
 */

#pragma once
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/GroundStation/GroundStation.h>

#include <Component/CommGS/Antenna.hpp>
#include <Library/math/MatVec.hpp>
#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>

using libra::Matrix;
using libra::Vector;

class GScalculator : public ILoggable {
 public:
  GScalculator(const double loss_polarization, const double loss_atmosphere, const double loss_rainfall, const double loss_others, const double EbN0,
               const double hardware_deterioration, const double coding_gain, const double margin_req);
  virtual ~GScalculator();
  void Update(const Dynamics& dynamics, const Antenna& sc_ant, const GroundStation& groundstation, const Antenna& gs_ant);

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline bool IsVisible() const { return is_visible_; }
  inline bool GetMaxBitrate() const { return max_bitrate_; }

 protected:
  double loss_polarization_;       //[dB]
  double loss_atmosphere_;         //[dB]
  double loss_rainfall_;           //[dB]
  double loss_others_;             //[dB]
  double EbN0_;                    //[dB]
  double hardware_deterioration_;  //[dB]
  double coding_gain_;             //[dB]
  double margin_req_;              //[dB]

  bool is_visible_;
  double max_bitrate_;  //[kbps]

  // Return true when the satellite is visible from the ground station
  bool CalcIsVisible(const Dynamics& dynamics, const GroundStation& groundstation);

  // Calculate the maximum bitrate
  double CalcMaxBitrate(const Dynamics& dynamics, const Antenna& sc_ant, const GroundStation& groundstation, const Antenna& gs_ant);
};
