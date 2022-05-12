#pragma once
#include "../math/Matrix.hpp"
#include "../math/Vector.hpp"

enum class RelativeOrbitModel { Hill = 0 };

enum class STMModel { HCW = 0 };

// Dynamics Models
libra::Matrix<6, 6> CalculateHillSystemMatrix(double orbit_radius, double mu);

// STMs
libra::Matrix<6, 6> CalculateHCWSTM(double orbit_radius, double mu, double elapsed_sec);