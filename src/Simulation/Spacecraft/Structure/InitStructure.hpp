#pragma once

#include <Simulation/Spacecraft/Structure/Structure.h>

KinematicsParams InitKinematicsParams(std::string ini_path);
vector<Surface> InitSurfaces(std::string ini_path);
RMMParams InitRMMParams(std::string ini_path);
