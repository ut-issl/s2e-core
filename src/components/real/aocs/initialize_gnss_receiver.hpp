/**
 * @file initialize_gnss_receiver.hpp
 * @brief Initialize functions for GNSS Receiver
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GNSS_RECEIVER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GNSS_RECEIVER_HPP_

#include <components/real/aocs/gnss_receiver.hpp>

/**
 * @fn InitGnssReceiver
 * @brief Initialize functions for GNSS Receiver without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] component_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simulation_time: Simulation time information
 */
GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, int component_id, const std::string fname, const Dynamics* dynamics,
                              const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time);
/**
 * @fn InitGnssReceiver
 * @brief Initialize functions for GNSS Receiver with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] component_id: Sensor ID
 * @param [in] power_port: Power port
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simulation_time: Simulation time information
 */
GnssReceiver InitGnssReceiver(ClockGenerator* clock_generator, PowerPort* power_port, int component_id, const std::string fname,
                              const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GNSS_RECEIVER_HPP_
