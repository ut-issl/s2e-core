/**
 * @file InitGnssReceiver.hpp
 * @brief Initialize functions for GNSS Receiver
 */
#pragma once

#include <Component/AOCS/GNSSReceiver.h>

/**
 * @fn InitGNSSReceiver
 * @brief Initialize functions for GNSS Receiver without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simtime: Simulation time information
 */
GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, int id, const std::string fname, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimTime* simtime);
/**
 * @fn InitGNSSReceiver
 * @brief Initialize functions for GNSS Receiver with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] id: Sensor ID
 * @param [in] power_port: Power port
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simtime: Simulation time information
 */
GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, PowerPort* power_port, int id, const std::string fname, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimTime* simtime);
