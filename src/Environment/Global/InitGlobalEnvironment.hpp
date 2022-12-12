/**
 *@file InitGlobalEnvironment.hpp
 *@brief Initialize functions for classes in global environment
 */

#pragma once

#include <Environment/Global/CelestialInformation.h>
#include <Environment/Global/HipparcosCatalogue.h>
#include <Environment/Global/SimTime.h>

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
