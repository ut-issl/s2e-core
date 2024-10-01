/*
 * @file ground_station_calculator.hpp
 * @brief Emulation of analysis and calculation for Ground Stations
 * @note TODO: This class is not `Component`. We need to move this to `Analysis` category and use this as library in future.
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_GROUND_STATION_CALCULATOR_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_GROUND_STATION_CALCULATOR_HPP_

#include <components/real/communication/antenna.hpp>
#include <dynamics/dynamics.hpp>
#include <environment/global/global_environment.hpp>
#include <logger/loggable.hpp>
#include <simulation/ground_station/ground_station.hpp>

namespace s2e::components {

/*
 * @class GroundStationCalculator
 * @brief Emulation of analysis and calculation for Ground Stations
 */
class GroundStationCalculator : public logger::ILoggable {
 public:
  /**
   * @fn GroundStationCalculator
   * @brief Constructor
   * @param [in] loss_polarization_dB: Loss polarization [dB]
   * @param [in] loss_atmosphere_dB: Loss atmosphere [dB]
   * @param [in] loss_rainfall_dB: Loss rainfall [dB]
   * @param [in] loss_others_dB: Loss others [dB]
   * @param [in] ebn0_dB: ebn0_dB [dB]
   * @param [in] hardware_deterioration_dB: Hardware deterioration [dB]
   * @param [in] coding_gain_dB: Coding gain [dB]
   * @param [in] margin_requirement_dB: Required margin to calculate max bitrate [dB]
   * @param [in] downlink_bitrate_bps: Downlink bitrate to calculate receive margin [bps]
   */
  GroundStationCalculator(const double loss_polarization_dB, const double loss_atmosphere_dB, const double loss_rainfall_dB,
                          const double loss_others_dB, const double EbN0, const double hardware_deterioration_dB, const double coding_gain_dB,
                          const double margin_requirement_dB, const double downlink_bitrate_bps = 1000);
  /**
   * @fn ~GroundStationCalculator
   * @brief Destructor
   */
  virtual ~GroundStationCalculator();

  /**
   * @fn Update
   * @brief Update maximum bitrate calculation
   * @param [in] spacecraft: Spacecraft information
   * @param [in] spacecraft_tx_antenna: Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] ground_station_rx_antenna: Antenna mounted on ground station
   */
  void Update(const simulation::Spacecraft& spacecraft, const Antenna& spacecraft_tx_antenna, const ground_station::GroundStation& ground_station,
              const Antenna& ground_station_rx_antenna);

  // Override logger::ILoggable TODO: Maybe we don't need logabble, and this class should be used as library.
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

  // Getter
  /**
   * @fn GetMaxBitrate_Mbps
   * @brief Return max bitrate [Mbps]
   */
  inline double GetMaxBitrate_Mbps() const { return max_bitrate_Mbps_; }
  /**
   * @fn GetReceiveMargin_dB
   * @brief Return receive margin [dB]
   */
  inline double GetReceiveMargin_dB() const { return receive_margin_dB_; }

  // Setter
  /**
   * @fn SetDownlinkBitrate_bps
   * @param [in] downlink_bitrate_bps: Downlink bitrate to calculate receive margin [bps]
   */
  inline void SetDownlinkBitrate_bps(const double downlink_bitrate_bps) { downlink_bitrate_bps_ = downlink_bitrate_bps; }

 protected:
  // Parameters
  double loss_polarization_dB_;       //!< Loss polarization [dB]
  double loss_atmosphere_dB_;         //!< Loss atmosphere [dB]
  double loss_rainfall_dB_;           //!< Loss rainfall [dB]
  double loss_others_dB_;             //!< Loss others [dB]
  double ebn0_dB_;                    //!< EbN0 (Energy per bit to Noise density ratio) [dB]
  double hardware_deterioration_dB_;  //!< Hardware deterioration [dB]
  double coding_gain_dB_;             //!< Coding gain [dB]
  // Variables
  double margin_requirement_dB_;  //!< Required margin to calculate max bitrate [dB]
  double downlink_bitrate_bps_;   //!< Downlink bitrate to calculate receive margin [bps]

  // Calculated values
  double receive_margin_dB_;  //!< Receive margin [dB]
  double max_bitrate_Mbps_;   //!< Max bitrate [Mbps]

  /**
   * @fn CalcMaxBitrate
   * @brief Calculate the maximum bitrate
   * @param [in] dynamics: Spacecraft dynamics information
   * @param [in] spacecraft_tx_antenna: Tx Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] ground_station_rx_antenna: Rx Antenna mounted on ground station
   * @return Max bitrate [Mbps]
   */
  double CalcMaxBitrate(const dynamics::Dynamics& dynamics, const Antenna& spacecraft_tx_antenna, const ground_station::GroundStation& ground_station,
                        const Antenna& ground_station_rx_antenna);
  /**
   * @fn CalcReceiveMarginOnGs
   * @brief Calculate receive margin at the ground station
   * @param [in] dynamics: Spacecraft dynamics information
   * @param [in] spacecraft_tx_antenna: Tx Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] ground_station_rx_antenna: Rx Antenna mounted on ground station
   * @return Receive margin [dB]
   */
  double CalcReceiveMarginOnGs(const dynamics::Dynamics& dynamics, const Antenna& spacecraft_tx_antenna,
                               const ground_station::GroundStation& ground_station, const Antenna& ground_station_rx_antenna);

  /**
   * @fn CalcCn0
   * @brief Calculate CN0 (Carrier to Noise density ratio) of received signal at the ground station
   * @param [in] dynamics: Spacecraft dynamics information
   * @param [in] spacecraft_tx_antenna: Tx Antenna mounted on spacecraft
   * @param [in] ground_station: Ground station information
   * @param [in] ground_station_rx_antenna: Rx Antenna mounted on ground station
   * @return CN0 [dB]
   */
  double CalcCn0OnGs(const dynamics::Dynamics& dynamics, const Antenna& spacecraft_tx_antenna, const ground_station::GroundStation& ground_station,
                     const Antenna& ground_station_rx_antenna);
};

/*
 * @fn InitGsCalculator
 * @brief Initialize function for Ground Station Calculator
 * @param [in] file_name: Path to initialize file
 */

GroundStationCalculator InitGsCalculator(const std::string file_name);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_GROUND_STATION_CALCULATOR_HPP_
