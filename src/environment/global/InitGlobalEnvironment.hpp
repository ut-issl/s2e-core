/**
 *@file InitGlobalEnvironment.hpp
 *@brief Initialize functions for classes in global environment
 */

#pragma once

#include <environment/global/celestial_information.hpp>
#include <environment/global/HipparcosCatalogue.h>
#include <environment/global/SimTime.h>

/**
 *@fn InitSimTime
 *@brief Initialize function for SimTime class
 *@param [in] file_name: Path to the initialize function
 */
SimTime* InitSimTime(std::string file_name);

/**
 *@fn InitHipCatalogue
 *@brief Initialize function for HipparcosCatalogue class
 *@param [in] file_name: Path to the initialize function
 */
HipparcosCatalogue* InitHipCatalogue(std::string file_name);

/**
 *@fn InitCelesInfo
 *@brief Initialize function for CelestialInformation class
 *@param [in] file_name: Path to the initialize function
 */
CelestialInformation* InitCelesInfo(std::string file_name);
