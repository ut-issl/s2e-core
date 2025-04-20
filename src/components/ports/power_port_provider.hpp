// filepath: s2e-core/src/components/ports/power_port_provider.hpp
#ifndef S2E_COMPONENTS_PORTS_POWER_PORT_PROVIDER_HPP_
#define S2E_COMPONENTS_PORTS_POWER_PORT_PROVIDER_HPP_

namespace s2e::components {

/**
 * @class PowerPortProvider
 * @brief Interface for providing power port information (e.g., power consumption)
 */
class PowerPortProvider {
 public:
  /**
   * @fn ~PowerPortProvider
   * @brief Virtual destructor
   */
  virtual ~PowerPortProvider() = default;

  /**
   * @fn GetPowerConsumption_W
   * @brief Get the power consumption for a specific port ID
   * @param [in] port_id: Power port ID
   * @return Power consumption in Watts. Returns 0.0 if the port is invalid or off.
   */
  virtual double GetPowerConsumption_W(const int port_id) const = 0;
};

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_PORTS_POWER_PORT_PROVIDER_HPP_