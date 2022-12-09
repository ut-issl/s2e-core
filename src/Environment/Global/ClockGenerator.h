/**
 * @file ClockGenerator.h
 * @brief Class to generate clock for classes which have ITickable
 */

#pragma once
#include <Component/Abstract/ITickable.h>

#include <vector>

#include "SimTime.h"

/**
 * @class ClockGenerator
 * @brief Class to generate clock for classes which have ITickable
 */
class ClockGenerator {
 public:
  /**
   * @fn ~ClockGenerator
   * @brief Destructor
   */
  ~ClockGenerator();

  /**
   * @fn RegisterComponent
   * @brief Register component which has ITickable
   * @param [in] ticlable: Component class
   */
  void RegisterComponent(ITickable* tickable);
  /**
   * @fn RemoveComponent
   * @brief Removed registered component
   * @param [in] ticlable: Registered component class
   */
  void RemoveComponent(ITickable* tickable);
  /**
   * @fn TickToComponents
   * @brief Execute tick function of all registered components
   */
  void TickToComponents();
  /**
   * @fn UpdateComponents
   * @brief Execute TickToComponents when component update timing
   * @param [in] sim_time: Simulation time
   */
  void UpdateComponents(const SimTime* sim_time);
  /**
   * @fn ClearTimerCount
   * @brief Clear time count
   */
  inline void ClearTimerCount(void) { timer_count_ = 0; }

  const int IntervalMillisecond = 1;  //!< Clock period [ms]. (Currenly, this is not used. TODO: Delete this.)

 private:
  std::vector<ITickable*> components_;  //!< Component list fot tick
  int timer_count_;                     //!< Timer count TODO: consider size, unsigned
};
