/*
 * @file Antenna.hpp
 * @brief Component emuration: RF antenna
 * @author 山本 智貴
 */

#pragma once
#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
using libra::Quaternion;
using libra::Vector;

class ANT {
 public:
  ANT(int id, const libra::Quaternion& q_b2c, bool is_transmitter, bool is_receiver, double frequency, Vector<4> tx_params, Vector<4> rx_params);
  ~ANT();
  void Initialize();

  // Calculations
  double CalcTxEIRP(double theta) const;
  double CalcRxGT(double theta) const;

  // Getter
  inline double GetFrequency() const { return frequency_; }
  inline bool IsTransmitter() const { return is_transmitter_; }
  inline bool IsReceiver() const { return is_receiver_; }

 protected:
  // General info
  int id_;               //! ID
  Quaternion q_b2c_;     //! corrdinate transform from body to component
  bool is_transmitter_;  //! Antenna for transmitter or not
  bool is_receiver_;     //! Antenna for receiver or not
  double frequency_;     //! Center Frequency [MHz]
  // Tx info
  double tx_output_;         //! RF output power [W]
  double tx_gain_;           //! transmit maximum gain [dBi]
  double tx_loss_feeder_;    //! feeder loss [dB]
  double tx_loss_pointing_;  //! pointing loss [dB]
  double tx_EIRP_;           //! transmit EIRP[dBW]
  // Rx info
  double rx_gain_;                      //! Receive maximum gain [dBi]
  double rx_loss_feeder_;               //! feeder loss [dB]
  double rx_loss_pointing_;             //! pointing loss [dB]
  double rx_system_noise_temperature_;  //! system noise temperature [K]
  double rx_GT_;                        //! receive G/T [dB/K]

  double CalcAntennaGain(double theta, bool is_tx) const;  //! Calc antenna gain [dB] considering the target direction
};
