#pragma once

#include <Disturbance/AirDrag.h>
#include <Disturbance/GGDist.h>
#include <Disturbance/GeoPotential.h>
#include <Disturbance/MagDisturbance.h>
#include <Disturbance/SolarRadiation.h>
#include <Disturbance/ThirdBodyGravity.h>

AirDrag InitAirDrag(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b);
SolarRadiation InitSRDist(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b);
GGDist InitGGDist(std::string ini_path);
MagDisturbance InitMagDisturbance(std::string ini_path, RMMParams rmm_params);
GeoPotential InitGeoPotential(std::string ini_path);
ThirdBodyGravity InitThirdBodyGravity(std::string ini_path, std::string ini_path_celes);
