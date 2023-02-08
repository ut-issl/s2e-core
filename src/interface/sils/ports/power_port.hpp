/**
 * @file power_port.hpp
 * @brief Class to emulate electrical power port
 */

#ifndef S2E_INTERFACE_SILS_PORTS_POWER_PORT_H_
#define S2E_INTERFACE_SILS_PORTS_POWER_PORT_H_

#include <string>

/**
 * @class PowerPort
 * @brief Class to emulate electrical power port
 * @details When the power switch is turned off, the component doesn't work same with the real world.
 */
class PowerPort {
 public:
  /**
   * @fn PowerPort
   * @brief Default Constructor for users who don't want to use this feature
   * @note The power switch is turned on with the default parameters
   */
  PowerPort();
  /**
   * @fn PowerPort
   * @brief Constructor
   * @param [in] port_id: ID of the power port
   * @param [in] current_Limit: Threshold to detect over current [A]
   */
  PowerPort(int port_id, double current_Limit);
  /**
   * @fn PowerPort
   * @brief Constructor
   * @param [in] port_id: ID of the power port
   * @param [in] current_Limit: Threshold to detect over current [A]
   * @param [in] minimum_voltage: Minimum voltage to work the component [V]
   * @param [in] assumed_power_consumption: Assumed power consumption of the component [W]
   */
  PowerPort(int port_id, double current_Limit, double minimum_voltage, double assumed_power_consumption);
  /**
   * @fn ~PowerPort
   * @brief Destructor
   */
  ~PowerPort();

  /**
   * @fn Update
   * @brief Update the power state considering the over current
   * @return Power switch state
   */
  bool Update(void);

  // Getters
  /**
   * @fn GetVoltage
   * @brief Return the voltage of this power line [V]
   */
  inline double GetVoltage(void) const { return voltage_; }
  /**
   * @fn GetCurrentConsumption
   * @brief Return the current consumption of this power line [A]
   */
  inline double GetCurrentConsumption() const { return current_consumption_; }
  /**
   * @fn GetAssumedPowerConsumption
   * @brief Return the assumed power consumption of this power line [W]
   */
  inline double GetAssumedPowerConsumption() const { return assumed_power_consumption_; }
  /**
   * @fn GetIsOn
   * @brief Return the power switch state
   */
  inline bool GetIsOn() const { return is_on_; }

  // Setters
  /**
   * @fn SetVoltage
   * @brief Set voltage to control the power switch state
   * @return Power switch state
   */
  bool SetVoltage(const double voltage);
  /**
   * @fn SetAssumedPowerConsumption
   * @brief Set assumed power consumption [W]
   * @note Users can use this function to change the power consumption of the component depending on the execution state.
   */
  inline void SetAssumedPowerConsumption(const double power) { assumed_power_consumption_ = power; }
  /**
   * @fn SetMinimumVoltage
   * @brief Set minimum voltage to work the component [V]
   */
  inline void SetMinimumVoltage(const double minimum_voltage) { minimum_voltage_ = minimum_voltage; }
  /**
   * @fn SetCurrentLimit
   * @brief Set threshold to detect over current [A]
   */
  inline void SetCurrentLimit(const double current_limit) { current_limit_ = current_limit; }

  // Others
  /**
   * @fn AddAssumedPowerConsumption
   * @brief Add assumed power consumption [W] to emulate power line which has multiple loads
   */
  inline void AddAssumedPowerConsumption(const double power) { assumed_power_consumption_ += power; }
  /**
   * @fn SubtractAssumedPowerConsumption
   * @brief Subtract assumed power consumption [W] to emulate power line which has multiple loads
   */
  void SubtractAssumedPowerConsumption(const double power);
  /**
   * @fn InitializeWithInitializeFile
   * @brief Initialize PowerPort class with initialize file
   */
  void InitializeWithInitializeFile(const std::string file_name);

 private:
  // PCU setting parameters
  const int kPortId;      //!< ID of the power port
  double current_limit_;  //!< Threshold to detect over current [A]

  // Components setting parameters
  double minimum_voltage_;            //!< Minimum voltage to work the component [V]
  double assumed_power_consumption_;  //!< Assumed power consumption of the component [W]

  double voltage_;              //!< Voltage of the power line[V]
  double current_consumption_;  //!< Current consumption calculated by I = P/V[A]
  bool is_on_;                  //!< Power switch state

  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(void);
};

#endif  // S2E_INTERFACE_SILS_PORTS_POWER_PORT_H_
