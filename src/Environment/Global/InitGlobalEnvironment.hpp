#pragma once

#include <Environment/Global/CelestialInformation.h>
#include <Environment/Global/HipparcosCatalogue.h>
#include <Environment/Global/SimTime.h>

SimTime* InitSimTime(std::string file_name);
HipparcosCatalogue* InitHipCatalogue(std::string file_name);
CelestialInformation* InitCelesInfo(std::string file_name);
