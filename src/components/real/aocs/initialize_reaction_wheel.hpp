/**
 * @file initialize_reaction_wheel.hpp
 * @brief Initialize functions for Reaction Wheel
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_REACTION_WHEEL_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_REACTION_WHEEL_HPP_

#include <components/real/aocs/reaction_wheel.hpp>

/**
 * @fn InitReactionWheel
 * @brief Initialize functions for reaction wheel without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step: Component step time [sec]
 */
ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, int actuator_id, std::string file_name, double prop_step, double compo_update_step);
/**
 * @fn InitReactionWheel
 * @brief Initialize functions for reaction wheel with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step: Component step time [sec]
 */
ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, std::string file_name, double prop_step,
                                double compo_update_step);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_REACTION_WHEEL_HPP_
