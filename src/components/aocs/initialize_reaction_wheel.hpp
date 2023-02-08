/**
 * @file initialize_reaction_wheel.hpp
 * @brief Initialize functions for Reaction Wheel
 */

#ifndef S2E_COMPONENTS_AOCS_INITIALIZE_REACTION_WHEEL_H_
#define S2E_COMPONENTS_AOCS_INITIALIZE_REACTION_WHEEL_H_

#include <components/aocs/reaction_wheel.hpp>

/**
 * @fn InitRWModel
 * @brief Initialize functions for reaction wheel without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step: Component step time [sec]
 */
RWModel InitRWModel(ClockGenerator* clock_gen, int actuator_id, std::string file_name, double prop_step, double compo_update_step);
/**
 * @fn InitRWModel
 * @brief Initialize functions for reaction wheel with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step: Component step time [sec]
 */
RWModel InitRWModel(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, std::string file_name, double prop_step,
                    double compo_update_step);

#endif  // S2E_COMPONENTS_AOCS_INITIALIZE_REACTION_WHEEL_H_
