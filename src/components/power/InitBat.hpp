/*
 * @file InitBAT.hpp
 * @brief Initialize function of BAT
 */
#pragma once

#include <components/power/battery.hpp>

/*
 * @fn InitBAT
 * @brief Initialize function of BAT
 * @param [in] clock_gen: Clock generator
 * @param [in] bat_id: Battery ID
 * @param [in] fname: Path to initialize file
 * @param [in] compo_step_time: Component step time [sec]
 */
BAT InitBAT(ClockGenerator* clock_gen, int bat_id, const std::string fname, double compo_step_time);
