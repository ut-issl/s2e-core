#pragma once

#include "SensorBase.h"

template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s);

#include "InitializeSensorBase_tfs.hpp"
