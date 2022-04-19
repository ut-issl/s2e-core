#pragma once

#include <Environment/Local/Atmosphere.h>
#include <Environment/Local/MagEnvironment.h>
#include <Environment/Local/SRPEnvironment.h>

MagEnvironment InitMagEnvironment(std::string ini_path);
SRPEnvironment InitSRPEnvironment(std::string ini_path, LocalCelestialInformation* local_celes_info);
Atmosphere InitAtmosphere(std::string ini_path);
