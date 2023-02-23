/**
 *@file initialize_global_environment.hpp
 *@brief Initialize functions for classes in global environment
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GLOBAL_ENVIRONMENT_HPP_
#define S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GLOBAL_ENVIRONMENT_HPP_

#include <environment/global/celestial_information.hpp>
#include <environment/global/hipparcos_catalogue.hpp>
#include <environment/global/simulation_time.hpp>

/**
 *@fn InitSimTime
 *@brief Initialize function for SimTime class
 *@param [in] file_name: Path to the initialize function
 */
SimTime* InitSimulationTime(std::string file_name);

/**
 *@fn InitHipCatalogue
 *@brief Initialize function for HipparcosCatalogue class
 *@param [in] file_name: Path to the initialize function
 */
HipparcosCatalogue* InitHipparcosCatalogue(std::string file_name);

/**
 *@fn InitCelesInfo
 *@brief Initialize function for CelestialInformation class
 *@param [in] file_name: Path to the initialize function
 */
CelestialInformation* InitCelestialInformation(std::string file_name);

#endif  // S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GLOBAL_ENVIRONMENT_HPP_
