/**
 * @file component.hpp
 * @brief Base class for component emulation. All components have to inherit this.
 */

#ifndef S2E_COMPONENTS_BASE_COMPONENT_HPP_
#define S2E_COMPONENTS_BASE_COMPONENT_HPP_

#include <components/ports/power_port.hpp>
#include <environment/global/clock_generator.hpp>
#include <utilities/macros.hpp>

#include "interface_tickable.hpp"

namespace s2e::components {

/**
 * @class Component
 * @brief Base class for component emulation. All components have to inherit this.
 * @details Component ha clock and power on/off features
 */
class Component : public ITickable {
 public:
  /**
   * @fn Component
   * @brief Constructor without power port
   * @note Power port is used as power on state
   * @param [in] prescaler: Frequency scale factor for normal update
   * @param [in] clock_generator: Clock generator
   * @param [in] fast_prescaler: Frequency scale factor for fast update (used only for component faster than component update period)
   */
  Component(const unsigned int prescaler, environment::ClockGenerator* clock_generator, const unsigned int fast_prescaler = 1);
  /**
   * @fn Component
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for normal update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] fast_prescaler: Frequency scale factor for fast update (used only for component faster than component update period)
   */
  Component(const unsigned int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const unsigned int fast_prescaler = 1);
  /**
   * @fn Component
   * @brief Copy constructor
   */
  Component(const Component& object);
  /**
   * @fn ~Component
   * @brief Destructor
   */
  virtual ~Component();

  // Override functions for ITickable
  /**
   * @fn Tick
   * @brief The methods to input clock. This will be called periodically.
   */
  virtual void Tick(const unsigned int count);
  /**
   * @fn FastTick
   * @brief The methods to input fast clock. This will be called periodically.
   */
  virtual void FastTick(const unsigned int fast_count);

 protected:
  unsigned int prescaler_;           //!< Frequency scale factor for normal update
  unsigned int fast_prescaler_ = 1;  //!< Frequency scale factor for fast update

  /**
   * @fn MainRoutine
   * @brief Pure virtual function periodically executed when the power switch is on.
   * @note The period is decided with the prescaler_ and the base clock.
   */
  virtual void MainRoutine(const int time_count) = 0;

  /**
   * @fn FastUpdate
   * @brief Pure virtual function used to calculate high-frequency disturbances(e.g. RW jitter)
   * @note Override only when high-frequency disturbances need to be calculated.
   */
  virtual void FastUpdate(){};

  /**
   * @fn PowerOffRoutine
   * @brief Pure virtual function executed when the power switch is off.
   */
  virtual void PowerOffRoutine(){};

  environment::ClockGenerator* clock_generator_;  //!< Clock generator
  PowerPort* power_port_;                         //!< Power port
};

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_BASE_COMPONENT_HPP_
