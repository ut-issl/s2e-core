#pragma once

#include <Disturbance/AirDrag.h>
#include <Disturbance/GeoPotential.h>
#include <Disturbance/MagDisturbance.h>
#include <Disturbance/SolarRadiation.h>
#include <Disturbance/ThirdBodyGravity.h>

#include <Disturbance/GravityGradient.hpp>

AirDrag InitAirDrag(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b);
SolarRadiation InitSRDist(std::string ini_path, const std::vector<Surface>& surfaces, const Vector<3> cg_b);

GravityGradient InitGravityGradient(std::string ini_path); // Center body is automatically set as the Earth
GravityGradient InitGravityGradient(std::string ini_path, const double mu_m3_s2);

MagDisturbance InitMagDisturbance(std::string ini_path, RMMParams rmm_params);
GeoPotential InitGeoPotential(std::string ini_path);
ThirdBodyGravity InitThirdBodyGravity(std::string ini_path, std::string ini_path_celes);
