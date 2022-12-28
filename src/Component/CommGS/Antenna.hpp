/*
 * @file Antenna.hpp
 * @brief Component emulation: RF antenna
 */

#pragma once
#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
using libra::Quaternion;
using libra::Vector;
#include <vector>

#include "./AntennaRadiationPattern.hpp"

/*
 * @enum AntennaGainModel
 * @brief Antenna gain model definition
 */
enum class AntennaGainModel {
  ISOTROPIC,              //!< Ideal isotropic antenna
  RADIATION_PATTERN_CSV,  //!< Radiation pattern obtained by CSV file
};

/*
 * @struct AntennaParameters
 * @brief Antenna parameters
 */
struct AntennaParameters {
  double gain_dBi_;                           //!< Transmit maximum gain [dBi]
  double loss_feeder_dB_;                     //!< Feeder loss [dB]
  double loss_pointing_dB_;                   //!< Pointing loss [dB]
  AntennaGainModel antenna_gain_model;        //!< Antenna gain model
  AntennaRadiationPattern radiation_pattern;  //!< Radiation pattern
};

/*
 * @class Antenna
 * @brief Component emulation: RF antenna
 */
class Antenna {
 public:
  /**
   * @fn Antenna
   * @brief Constructor
   * @param [in] id: Antenna ID
   * @param [in] q_b2c: Coordinate transform from body to component
   * @param [in] is_transmitter: Antenna for transmitter or not
   * @param [in] is_receiver: Antenna for receiver or not
   * @param [in] frequency: Center Frequency [MHz]
   * @param [in] tx_params: output, gain, loss_feeder, loss_pointing for TX
   * @param [in] rx_params: gain, loss_feeder, loss_pointing, system_temperature for RX
   *
   */
  Antenna(const int id, const libra::Quaternion& q_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
          const Vector<4> tx_params, const Vector<4> rx_params);
  /**
   * @fn ~Antenna
   * @brief Destructor
   */
  ~Antenna();

  /**
   * @fn CalcTxEIRP
   * @brief Calculation of TX EIRP
   * @param [in] theta: Target direction angle [rad]
   * @return TX EIRP [dBW]
   */
  double CalcTxEIRP(double theta) const;
  /**
   * @fn CalcRxGT
   * @brief Calculation of RX G/T
   * @param [in] theta: Target direction angle [rad]
   * @return RX G/T [dB/K]
   */
  double CalcRxGT(double theta) const;

  // Getter
  /**
   * @fn GetFrequency
   * @brief Return frequency [MHz]
   */
  inline double GetFrequency() const { return frequency_; }
  /**
   * @fn IsTransmitter
   * @brief Return antenna for transmitter or not
   */
  inline bool IsTransmitter() const { return is_transmitter_; }
  /**
   * @fn IsReceiver
   * @brief Return antenna for receiver or not
   */
  inline bool IsReceiver() const { return is_receiver_; }

 protected:
  // General info
  int id_;               //!< Antenna ID
  Quaternion q_b2c_;     //!< Coordinate transform from body to component
  bool is_transmitter_;  //!< Antenna for transmitter or not
  bool is_receiver_;     //!< Antenna for receiver or not
  double frequency_;     //!< Center Frequency [MHz]

  // Tx info
  AntennaParameters tx_params_;  //!< Tx parameters
  double tx_output_power_W_;     //!< Transmit output power [W]
  double tx_eirp_;               //!< Transmit EIRP(Equivalent Isotropic Radiated Power) [dBW]

  // Rx info
  AntennaParameters rx_params_;           //!< Rx parameters
  double rx_system_noise_temperature_K_;  //!< Receive system noise temperature [K]
  double rx_gt_;                          //!< Receive G/T [dB/K]

  /**
   * @fn CalcAntennaGain
   * @brief Calculation antenna gain considering the target direction
   * @param [in] theta: Target direction angle [rad]
   * @param [in] is_tx: Flag TX(True) or RX(False)
   * @return Antenna gain [dB]
   */
  double CalcAntennaGain(double theta, bool is_tx) const;
};
