/*
 * @file InitSimpleThruster.hpp
 * @brief Initialize function os SimpleThruster
 */
#pragma once

#include <Component/Propulsion/SimpleThruster.h>

/**
 * @fn InitSimpleThruster
 * @brief Initialize function os SimpleThruster
 * @param [in] clock_gen: Clock generator
 * @param [in] thruster_id: Thruster ID
 * @param [in] fname: Path to initialize file
 * @param [in] structure: Spacecraft structure information
 * @param [in] dynamics: Spacecraft dynamics information
 */
SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, int thruster_id, const std::string fname, const Structure* structure,
                                  const Dynamics* dynamics);
/**
 * @fn InitSimpleThruster
 * @brief Initialize function os SimpleThruster
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] thruster_id: Thruster ID
 * @param [in] fname: Path to initialize file
 * @param [in] structure: Spacecraft structure information
 * @param [in] dynamics: Spacecraft dynamics information
 */
SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, PowerPort* power_port, int thruster_id, const std::string fname,
                                  const Structure* structure, const Dynamics* dynamics);
