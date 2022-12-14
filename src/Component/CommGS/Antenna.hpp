/*
 * @file Antenna.hpp
 * @brief Component emulation: RF antenna
 */

#pragma once
#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
using libra::Quaternion;
using libra::Vector;

/*
 * @class Antenna
 * @brief Component emulation: RF antenna
 */
class Antenna {
 public:
  /**
   * @fn Antenna
   * @brief Constructor
   * @param [in] clock_gen: Clock generator
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
   * @param theta: Target direction angle [rad]
   * @return TX EIRP [dBW]
   */
  double CalcTxEIRP(double theta) const;
  /**
   * @fn CalcRxGT
   * @brief Calculation of RX G/T
   * @param theta: Target direction angle [rad]
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
  double tx_output_;         //!< RF output power [W]
  double tx_gain_;           //!< Transmit maximum gain [dBi]
  double tx_loss_feeder_;    //!< Feeder loss [dB]
  double tx_loss_pointing_;  //!< Pointing loss [dB]
  double tx_EIRP_;           //!< Transmit EIRP(Equivalent Isotropic Radiated Power) [dBW]
  // Rx info
  double rx_gain_;                      //!< Receive maximum gain [dBi]
  double rx_loss_feeder_;               //!< Feeder loss [dB]
  double rx_loss_pointing_;             //!< Pointing loss [dB]
  double rx_system_noise_temperature_;  //!< System noise temperature [K]
  double rx_GT_;                        //!< Receive G/T [dB/K]

  /**
   * @fn CalcAntennaGain
   * @brief Calculation antenna gain considering the target direction
   * @param [in] theta: Target direction angle [rad]
   * @param [in] is_tx: Flag TX(True) or RX(False)
   * @return Antenna gain [dB]
   */
  double CalcAntennaGain(double theta, bool is_tx) const;
};
