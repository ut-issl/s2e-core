/*
 * @file antenna.hpp
 * @brief Component emulation: RF antenna
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_ANTENNA_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_ANTENNA_HPP_

#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
using libra::Quaternion;
using libra::Vector;
#include <vector>

#include "./antenna_radiation_pattern.hpp"

/*
 * @enum AntennaGainModel
 * @brief Antenna gain model definition
 */
enum class AntennaGainModel {
  kIsotropic,            //!< Ideal isotropic antenna
  kRadiationPatternCsv,  //!< Radiation pattern obtained by CSV file
};

/*
 * @struct AntennaParameters
 * @brief Antenna parameters
 */
struct AntennaParameters {
  double gain_dBi_;                           /*!< Gain used in kIsotropic mode [dBi]
                                                   Generally, it is zero but users can set any value for ideal analysis */
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
   * @note TODO: This constructor will be removed.
   * @param [in] component_id: Antenna ID
   * @param [in] quaternion_b2c: Coordinate transform from body to component
   * @param [in] is_transmitter: Antenna for transmitter or not
   * @param [in] is_receiver: Antenna for receiver or not
   * @param [in] frequency_MHz: Center Frequency [MHz]
   * @param [in] tx_parameters: output, gain, loss_feeder, loss_pointing for TX
   * @param [in] rx_parameters: gain, loss_feeder, loss_pointing, system_temperature for RX
   */
  Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
          const double frequency_MHz, const Vector<5> tx_parameters, const Vector<4> rx_parameters);

  /**
   * @fn Antenna
   * @brief Constructor
   * @param [in] component_id: Antenna ID
   * @param [in] quaternion_b2c: Coordinate transform from body to component
   * @param [in] is_transmitter: Antenna for transmitter or not
   * @param [in] is_receiver: Antenna for receiver or not
   * @param [in] frequency_MHz: Center Frequency [MHz]
   * @param [in] tx_bitrate_bps: Transmit bitrate [bps]
   * @param [in] tx_output_power_W: Transmit output power [W]
   * @param [in] tx_parameters: TX antenna parameters
   * @param [in] rx_system_noise_temperature_K: Receive system noise temperature [K]
   * @param [in] rx_parameters: RX antenna parameters
   */
  Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
          const double frequency_MHz, const double tx_bitrate_bps, const double tx_output_power_W, const AntennaParameters tx_parameters,
          const double rx_system_noise_temperature_K, const AntennaParameters rx_parameters);
  /**
   * @fn ~Antenna
   * @brief Destructor
   */
  ~Antenna();

  /**
   * @fn CalcTxEirp_dBW
   * @brief Calculation of TX EIRP
   * @param [in] theta: Angle from PZ axis on the antenna frame [rad]
   * @param [in] phi: from PX axis on the antenna frame [rad] (Set zero for axial symmetry pattern)
   * @return TX EIRP [dBW]
   */
  double CalcTxEirp_dBW(const double theta_rad, const double phi_rad = 0.0) const;
  /**
   * @fn CalcRxGt_dB_K
   * @brief Calculation of RX G/T
   * @param [in] theta: Angle from PZ axis on the antenna frame [rad]
   * @param [in] phi: from PX axis on the antenna frame [rad] (Set zero for axial symmetry pattern)
   * @return RX G/T [dB/K]
   */
  double CalcRxGt_dB_K(const double theta_rad, const double phi_rad = 0.0) const;

  // Getter
  /**
   * @fn GetFrequency_MHz
   * @brief Return frequency_MHz [MHz]
   */
  inline double GetFrequency_MHz() const { return frequency_MHz_; }
  /**
   * @fn GetBitrate
   * @brief Return bitrate [bps]
   */
  inline double GetBitrate_bps() const { return tx_bitrate_bps_; }
  /**
   *
   * @fn GetQuaternion_b2c
   * @brief Return quaternion from body to component
   */
  inline Quaternion GetQuaternion_b2c() const { return quaternion_b2c_; }

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
  int component_id_;           //!< Antenna ID
  Quaternion quaternion_b2c_;  //!< Coordinate transform from body to component
  bool is_transmitter_;        //!< Antenna for transmitter or not
  bool is_receiver_;           //!< Antenna for receiver or not
  double frequency_MHz_;       //!< Center Frequency [MHz]

  // Tx info
  double tx_bitrate_bps_;            //!< Transmit bitrate [bps]
  double tx_output_power_W_;         //!< Transmit output power [W]
  AntennaParameters tx_parameters_;  //!< Tx parameters
  double tx_eirp_dBW_;               //!< Transmit EIRP(Equivalent Isotropic Radiated Power) [dBW]

  // Rx info
  double rx_system_noise_temperature_K_;  //!< Receive system noise temperature [K]
  AntennaParameters rx_parameters_;       //!< RX parameters
  double rx_gt_dBK_;                      //!< Receive G/T(Gain/Temperature) [dB/K]

  /**
   * @fn CalcAntennaGain
   * @brief Calculation antenna gain considering the target direction
   * @param [in] antenna_parameters: Antenna parameters
   * @param [in] theta_rad: Angle from PZ axis on the antenna frame [rad]
   * @param [in] phi_rad: from PX axis on the antenna frame [rad] (Set zero for axial symmetry pattern)
   * @return Antenna gain [dBi]
   */
  double CalcAntennaGain(const AntennaParameters antenna_parameters, const double theta_rad, const double phi_rad = 0.0) const;
};

AntennaGainModel SetAntennaGainModel(const std::string gain_model_name);

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_ANTENNA_HPP_
