/*
 * @file GScalculator.h
 * @brief Emulation of analysis and calculation for Ground Stations
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

/*
 * @class GScalculator
 * @brief Emulation of analysis and calculation for Ground Stations
 */
class GScalculator : public ILoggable {
 public:
  /**
   * @fn GScalculator
   * @brief Constructor
   * @param [in] loss_polarization: Loss polarization [dB]
   * @param [in] loss_atmosphere: Loss atmosphere [dB]
   * @param [in] loss_rainfall: Loss rainfall [dB]
   * @param [in] loss_others: Loss others [dB]
   * @param [in] EbN0: EbN0 [dB]
   * @param [in] hardware_deterioration: Hardware deterioration [dB]
   * @param [in] coding_gain: Coding gain [dB]
   * @param [in] margin_req: Margin requirement [dB]
   */
  GScalculator(const double loss_polarization, const double loss_atmosphere, const double loss_rainfall, const double loss_others, const double EbN0,
               const double hardware_deterioration, const double coding_gain, const double margin_req);
  /**
   * @fn ~GScalculator
   * @brief Destructor
   */
  virtual ~GScalculator();

  /**
   * @fn Update
   * @brief Update state
   * @param [in] spacecraft: Spacecraft information
   * @param [in] sc_ant: Antenna mounted on spacecraft
   * @param [in] groundstation: Ground station information
   * @param [in] gs_ant: Antenna mounted on ground station
   * @return Max bitrate [kbps]
   */
  void Update(const Spacecraft& spacecraft, const Antenna& sc_ant, const GroundStation& groundstation, const Antenna& gs_ant);

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn GetMaxBitrate
   * @brief Return max bitrate [kbps]
   */
  inline bool GetMaxBitrate() const { return max_bitrate_; }

 protected:
  double loss_polarization_;       //!< Loss polarization [dB]
  double loss_atmosphere_;         //!< Loss atmosphere [dB]
  double loss_rainfall_;           //!< Loss rainfall [dB]
  double loss_others_;             //!< Loss others [dB]
  double EbN0_;                    //!< EbN0 [dB]
  double hardware_deterioration_;  //!< Hardware deterioration [dB]
  double coding_gain_;             //!< Coding gain [dB]
  double margin_req_;              //!< Margin requirement [dB]
  double max_bitrate_;             //!< Max bitrate [kbps]

  /**
   * @fn CalcMaxBitrate
   * @brief Calculate the maximum bitrate
   * @param [in] dynamics: Spacecraft dynamics information
   * @param [in] sc_tx_ant: Tx Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] gs_rx_ant: Rx Antenna mounted on ground station
   * @return Max bitrate [kbps]
   */
  double CalcMaxBitrate(const Dynamics& dynamics, const Antenna& sc_tx_ant, const GroundStation& ground_station, const Antenna& gs_rx_ant);

  /**
   * @fn CalcCn0
   * @brief Calculate CN0 (Carrier to Noise density ratio) of received signal at the ground station
   * @param [in] dynamics: Spacecraft dynamics information
   * @param [in] sc_tx_ant: Tx Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] gs_rx_ant: Rx Antenna mounted on ground station
   * @return CN0 [dB]
   */
  double CalcCn0OnGs(const Dynamics& dynamics, const Antenna& sc_tx_ant, const GroundStation& ground_station, const Antenna& gs_rx_ant);
};
